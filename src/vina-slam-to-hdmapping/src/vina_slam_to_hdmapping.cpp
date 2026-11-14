// Converts the output VINA-SLAM writes itself to an HDMapping session
// (chunked LAZ + trajectory CSVs + poses + session.json).
//
// VINA-SLAM reads the bag on its own and, with is_save_pose and is_save_map
// on, writes into its session folder:
//   trajectory.txt  one TUM row "t x y z qx qy qz qw" per frame: the pose of the
//                   IMU/body frame in the world frame after the local BA, and t
//                   the scan-end time of that frame (bag time)
//   pcd/NNNNNN.pcd  the motion-compensated points of frame NNNNNN in its own
//                   body frame; frame NNNNNN is row NNNNNN of trajectory.txt
//
// Each frame's points are placed in the world with the pose of its row. Frames
// are processed in order and a chunk is written as soon as it holds
// CHUNK_POINTS points, so memory stays bounded however dense the frames are.
// A chunk's points and trajectory are stored relative to its first pose, as in
// the other benchmark converters.

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "laz_writer.hpp"

namespace fs = std::filesystem;

namespace
{

constexpr size_t CHUNK_POINTS = 2000000;

struct TrajectoryPose
{
    uint64_t timestamp_ns;
    Eigen::Affine3d pose;
    double om_rad;  // Roll (omega)
    double fi_rad;  // Pitch (phi)
    double ka_rad;  // Yaw (kappa)
};

struct TaitBryanPose
{
    double px;
    double py;
    double pz;
    double om;
    double fi;
    double ka;
};

TaitBryanPose pose_tait_bryan_from_affine_matrix(const Eigen::Affine3d &m)
{
    TaitBryanPose pose;

    pose.px = m(0, 3);
    pose.py = m(1, 3);
    pose.pz = m(2, 3);

    if (m(0, 2) < 1) {
        if (m(0, 2) > -1) {
            pose.fi = asin(m(0, 2));
            pose.om = atan2(-m(1, 2), m(2, 2));
            pose.ka = atan2(-m(0, 1), m(0, 0));
            return pose;
        }
        pose.fi = -M_PI / 2.0;
        pose.om = -atan2(m(1, 0), m(1, 1));
        pose.ka = 0;
        return pose;
    }
    pose.fi = M_PI / 2.0;
    pose.om = atan2(m(1, 0), m(1, 1));
    pose.ka = 0.0;
    return pose;
}

// "1725010575.012345678" -> 1725010575012345678, digit by digit: a double
// cannot hold a Unix time with nanoseconds exactly.
bool parse_timestamp_ns(const std::string &text, uint64_t &ns)
{
    const size_t dot = text.find('.');
    const std::string sec_part = text.substr(0, dot);
    std::string frac_part = (dot == std::string::npos) ? "" : text.substr(dot + 1);
    if (sec_part.empty() || sec_part.find_first_not_of("0123456789") != std::string::npos ||
        frac_part.find_first_not_of("0123456789") != std::string::npos) {
        return false;
    }
    frac_part = (frac_part + "000000000").substr(0, 9);
    ns = std::stoull(sec_part) * 1000000000ULL + std::stoull(frac_part);
    return true;
}

bool read_trajectory(const fs::path &path, std::vector<TrajectoryPose> &trajectory)
{
    std::ifstream in(path);
    if (!in.good()) {
        std::cerr << "Error: cannot open " << path << std::endl;
        return false;
    }
    std::string line;
    size_t line_number = 0;
    size_t skipped = 0;
    while (std::getline(in, line)) {
        line_number++;
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream fields(line);
        std::string t;
        double x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 1;
        TrajectoryPose pose;
        if (!(fields >> t >> x >> y >> z >> qx >> qy >> qz >> qw) || !parse_timestamp_ns(t, pose.timestamp_ns)) {
            std::cerr << "WARNING: " << path.filename().string() << " line " << line_number
                      << " is not a TUM row, skipped: " << line << std::endl;
            skipped++;
            // Keep the row so that row N still matches pcd/N.pcd.
            pose.timestamp_ns = 0;
        }
        pose.pose = Eigen::Affine3d::Identity();
        pose.pose.translation() = Eigen::Vector3d(x, y, z);
        pose.pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
        const TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(pose.pose);
        pose.om_rad = tb.om;
        pose.fi_rad = tb.fi;
        pose.ka_rad = tb.ka;
        trajectory.push_back(pose);
    }
    if (skipped > 0) {
        std::cerr << "WARNING: " << skipped << " unreadable trajectory rows; their frames are left out" << std::endl;
    }
    return true;
}

std::string frame_pcd_name(size_t frame)
{
    std::ostringstream name;
    name << std::setfill('0') << std::setw(6) << frame << ".pcd";
    return name.str();
}

// Reads x, y, z and intensity of one frame (VINA-SLAM saves PointXYZINormal).
bool read_frame(const fs::path &path, pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    try {
        pcl::PCLPointCloud2 blob;
        if (pcl::io::loadPCDFile(path.string(), blob) != 0) {
            return false;
        }
        pcl::fromPCLPointCloud2(blob, cloud);
        return true;
    } catch (const std::exception &e) {
        std::cerr << "WARNING: cannot read " << path << ": " << e.what() << std::endl;
        return false;
    }
}

bool save_poses(const fs::path &file_name, const std::vector<Eigen::Affine3d> &m_poses,
                const std::vector<std::string> &filenames)
{
    std::ofstream outfile(file_name);
    if (!outfile.good()) {
        std::cerr << "can not save file: " << file_name << std::endl;
        return false;
    }

    outfile << m_poses.size() << std::endl;
    for (size_t i = 0; i < m_poses.size(); i++) {
        outfile << filenames[i] << std::endl;
        outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
        outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
        outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
        outfile << "0 0 0 1" << std::endl;
    }
    return outfile.good();
}

struct Chunk
{
    std::vector<Point3Di> points;  // world frame until the chunk is written
    std::vector<TrajectoryPose> trajectory;
};

struct Session
{
    fs::path folder;
    std::vector<Eigen::Affine3d> m_poses;
    std::vector<std::string> laz_names;
    Eigen::Vector3d position_sum = Eigen::Vector3d::Zero();
    size_t position_count = 0;
};

// Writes scan_lio_<index>.laz and trajectory_lio_<index>.csv, both relative to
// the chunk's first pose.
bool write_chunk(Chunk &chunk, size_t index, Session &session)
{
    const Eigen::Affine3d first_affine = chunk.trajectory.front().pose;
    const Eigen::Affine3d first_affine_inv = first_affine.inverse();

    for (auto &p : chunk.points) {
        p.point = first_affine_inv * p.point;
    }

    const std::string laz_name = "scan_lio_" + std::to_string(index) + ".laz";
    const fs::path laz_path = session.folder / laz_name;
    std::cout << "saving to: " << laz_path << " number of points: " << chunk.points.size() << std::endl;
    if (!saveLaz(laz_path.string(), chunk.points)) {
        std::cerr << "Error: cannot write " << laz_path << std::endl;
        return false;
    }

    const fs::path csv_path = session.folder / ("trajectory_lio_" + std::to_string(index) + ".csv");
    std::cout << "saving to: " << csv_path << std::endl;
    std::ofstream outfile(csv_path);
    if (!outfile.good()) {
        std::cerr << "Error: cannot write " << csv_path << std::endl;
        return false;
    }
    outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds om_rad fi_rad ka_rad" << std::endl;
    for (const auto &tp : chunk.trajectory) {
        const Eigen::Affine3d pose = first_affine_inv * tp.pose;
        outfile << tp.timestamp_ns << " " << std::setprecision(10)
                << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " " << pose(0, 3) << " "
                << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " " << pose(1, 3) << " "
                << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << " "
                << tp.timestamp_ns << " " << std::setprecision(20)
                << tp.om_rad << " " << tp.fi_rad << " " << tp.ka_rad << " " << std::endl;
        session.position_sum += tp.pose.translation();
        session.position_count++;
    }
    if (!outfile.good()) {
        std::cerr << "Error: cannot write " << csv_path << std::endl;
        return false;
    }

    session.m_poses.push_back(first_affine);
    session.laz_names.push_back(laz_name);
    return true;
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <vina_slam_session_dir> <output_directory>" << std::endl;
        std::cout << "  vina_slam_session_dir: the folder with VINA-SLAM's trajectory.txt and pcd/" << std::endl;
        return 1;
    }

    const fs::path input_dir = argv[1];
    const fs::path output_dir = argv[2];
    const fs::path trajectory_file = input_dir / "trajectory.txt";
    const fs::path pcd_dir = input_dir / "pcd";

    std::cout << "VINA-SLAM session : " << input_dir << std::endl;
    std::cout << "Output            : " << output_dir << std::endl;

    std::vector<TrajectoryPose> trajectory;
    if (!read_trajectory(trajectory_file, trajectory)) {
        return 1;
    }
    std::cout << "Read " << trajectory.size() << " trajectory rows." << std::endl;
    if (trajectory.empty()) {
        std::cerr << "Error: " << trajectory_file << " has no poses; VINA-SLAM produced no trajectory." << std::endl;
        return 1;
    }

    try {
        fs::create_directories(output_dir);
    } catch (const fs::filesystem_error &e) {
        std::cerr << "Error creating directory: " << e.what() << std::endl;
        return 1;
    }

    Session session;
    session.folder = output_dir;

    Chunk chunk;
    size_t chunk_index = 0;
    size_t frames_with_points = 0;
    size_t missing_frames = 0;
    size_t total_points = 0;

    for (size_t frame = 0; frame < trajectory.size(); frame++) {
        const TrajectoryPose &tp = trajectory[frame];
        if (tp.timestamp_ns == 0) {
            continue;  // unreadable row, reported while reading
        }
        if (frame % 1000 == 0) {
            std::cout << "frame [" << frame + 1 << "] of: " << trajectory.size() << std::endl;
        }

        chunk.trajectory.push_back(tp);

        pcl::PointCloud<pcl::PointXYZI> cloud;
        const fs::path pcd_path = pcd_dir / frame_pcd_name(frame);
        if (!read_frame(pcd_path, cloud)) {
            if (missing_frames < 5) {
                std::cerr << "WARNING: no points for frame " << frame << " (" << pcd_path << ")" << std::endl;
            }
            missing_frames++;
            continue;
        }

        const double timestamp = static_cast<double>(tp.timestamp_ns);
        for (size_t i = 0; i < cloud.size(); i++) {
            const auto &p = cloud.points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                continue;
            }
            Point3Di point;
            point.point = tp.pose * Eigen::Vector3d(p.x, p.y, p.z);
            point.timestamp = timestamp;
            point.intensity = p.intensity;
            point.index_pose = static_cast<int>(i);
            point.lidarid = 0;
            point.index_point = static_cast<int>(i);
            chunk.points.push_back(point);
        }
        total_points += cloud.size();
        frames_with_points++;

        if (chunk.points.size() >= CHUNK_POINTS) {
            if (!write_chunk(chunk, chunk_index, session)) {
                return 1;
            }
            chunk_index++;
            chunk = Chunk();
        }
    }

    if (!chunk.points.empty()) {
        if (!write_chunk(chunk, chunk_index, session)) {
            return 1;
        }
    } else if (!chunk.trajectory.empty()) {
        std::cerr << "WARNING: the last " << chunk.trajectory.size()
                  << " poses have no points after them and are left out" << std::endl;
    }

    if (missing_frames > 0) {
        std::cerr << "WARNING: " << missing_frames << " of " << trajectory.size()
                  << " frames have no readable PCD in " << pcd_dir << std::endl;
    }
    std::cout << "Frames with points: " << frames_with_points << ", points: " << total_points
              << ", chunks: " << session.laz_names.size() << std::endl;

    if (session.laz_names.empty()) {
        std::cerr << "Error: no frame had readable points in " << pcd_dir << std::endl;
        return 1;
    }

    // The chunk poses are shifted by the mean position, as in the other converters.
    const Eigen::Vector3d offset = session.position_sum / static_cast<double>(session.position_count);
    for (auto &m : session.m_poses) {
        m.translation() -= offset;
    }

    if (!save_poses(output_dir / "lio_initial_poses.reg", session.m_poses, session.laz_names) ||
        !save_poses(output_dir / "poses.reg", session.m_poses, session.laz_names)) {
        return 1;
    }

    const fs::path session_file = output_dir / "session.json";
    std::cout << "saving file: " << session_file << std::endl;

    nlohmann::json jj;
    nlohmann::json j;
    j["offset_x"] = 0.0;
    j["offset_y"] = 0.0;
    j["offset_z"] = 0.0;
    j["folder_name"] = output_dir.string();
    j["out_folder_name"] = output_dir.string();
    j["poses_file_name"] = (output_dir / "poses.reg").string();
    j["initial_poses_file_name"] = (output_dir / "lio_initial_poses.reg").string();
    j["out_poses_file_name"] = (output_dir / "poses.reg").string();
    j["lidar_odometry_version"] = "HdMap";
    jj["Session Settings"] = j;

    nlohmann::json jlaz_file_names = nlohmann::json::array();
    for (const auto &name : session.laz_names) {
        jlaz_file_names.push_back({{"file_name", (output_dir / name).string()}});
    }
    jj["laz_file_names"] = jlaz_file_names;

    std::ofstream session_out(session_file);
    session_out << jj.dump(2);
    if (!session_out.good()) {
        std::cerr << "Error: cannot write " << session_file << std::endl;
        return 1;
    }

    return 0;
}
