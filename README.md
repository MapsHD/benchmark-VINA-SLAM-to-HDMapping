## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-VINA-SLAM-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.

## Example Dataset:

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)

# benchmark-VINA-SLAM-to-HDMapping

Runs [VINA-SLAM](https://github.com/SheepYang666/VINA-SLAM) on a rosbag and
converts the output to an [HDMapping](https://github.com/MapsHD/HDMapping)
session.

VINA-SLAM (R. Zhang, B. Sun, "VINA-SLAM: A Voxel-Based Inertial and
Normal-Aligned LiDAR–IMU SLAM", Sensors 2026) is a LiDAR-inertial SLAM built on
a global voxel map. Its sliding-window bundle adjustment couples IMU factors,
planar constraints and a normal-consistency factor between the scan and map
plane normals.

## Prerequisites

- Docker
- A ROS 2 bag directory, or a ROS 1 bag file (its LiDAR and IMU topics are
  converted to ROS 2 automatically), with a LiDAR topic and a
  `sensor_msgs/Imu` topic. The LiDAR message depends on `lidar_type` in the
  parameter file:

| `lidar_type` | LiDAR message |
|--------------|---------------|
| `0` (Livox CustomMsg) | `livox_ros_driver2/msg/CustomMsg` |
| `1`–`7` (Velodyne, Ouster, Hesai, RoboSense, Simulation, RoboSense Airy, Livox MID360 PointCloud2) | `sensor_msgs/msg/PointCloud2` |

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-VINA-SLAM-to-HDMapping.git --recursive
cd benchmark-VINA-SLAM-to-HDMapping
```

## Step 2 — Build the Docker image

```bash
docker build -t vina-slam_humble .
```

This installs:
- Ubuntu 22.04 + ROS 2 Humble, PCL
- Livox-SDK2 and `livox_ros_driver2`, for the Livox `CustomMsg`
- colcon workspace with `vina_slam` and `vina_slam_to_hdmapping`

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-ros2-vina-slam.sh
./docker_session_run-ros2-vina-slam.sh /path/to/input /path/to/output/dir
```

The input is a ROS 2 bag directory or a ROS 1 `.bag` file. With no arguments, a
GUI file selector is used (requires `zenity`).

| Variable | Default | Meaning |
|----------|---------|---------|
| `CONFIG_FILE` | `mid360_bench.yaml` | VINA-SLAM parameter file: a file on the host, or the name of one in VINA-SLAM's [config/](https://github.com/SheepYang666/VINA-SLAM/tree/main/config) folder |
| `USE_RVIZ` | `1` | RViz live view |

`mid360_bench.yaml` is VINA-SLAM's `mid360.yaml` (Livox Mid-360:
`/livox/lidar` as `CustomMsg` and `/livox/imu`) with the local bundle
adjustment on, see [Notes on VINA-SLAM](#notes-on-vina-slam).

To use your own sensor, copy a parameter file, set `lid_topic`, `imu_topic`,
`lidar_type` and the LiDAR-to-IMU extrinsics, and pass it:

```bash
CONFIG_FILE=/path/to/my_sensor.yaml \
  ./docker_session_run-ros2-vina-slam.sh /path/to/input /path/to/output/dir
```

Before anything starts, the script checks that the parameter file's topics are
in the bag with a type VINA-SLAM reads for that `lidar_type`.

**What happens:**

VINA-SLAM reads the bag itself (there is no `ros2 bag play`), as fast as it
processes the scans, and exits when the bag is done. The script opens a Docker
container with a tmux session containing three panes on window 0 and a
`control` window (window 1, the attach target):

| Pane | Role |
|------|------|
| 0 | `ros2 launch vina_slam vina_slam_bench.launch.py` — VINA-SLAM, reading the bag |
| 1 | RViz live view |
| 2 | diagnostics — shows active topics, publishing rates and the frames saved so far |

When VINA-SLAM exits, the control window stops RViz and exits tmux. A second
Docker run then converts VINA-SLAM's output into the HDMapping session format.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-vina-slam/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
...
session.json
trajectory_lio_0.csv
...
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping)
application.

VINA-SLAM's own output stays in `<output_dir>/vina-slam-results/` and the logs
in `<output_dir>/logs/`.

## Notes on VINA-SLAM

**Bundle adjustment on.** VINA-SLAM's `mid360.yaml` has `if_BA: 0`, which
runs the EKF odometry only, without the sliding-window bundle adjustment and
its normal-consistency factor. The benchmark's `mid360_bench.yaml` changes only
that line to `if_BA: 1`, as in VINA-SLAM's other parameter files (HILTI,
velodyne, robosense, outdoor_fly, compus_elevator). The Docker build creates it
from `mid360.yaml` and checks that nothing else differs.

**Output.** The benchmark turns on VINA-SLAM's own saving, so nothing is
recorded from topics:

| File | Meaning |
|------|---------|
| `vina-slam-results/trajectory.txt` | one TUM row per frame: the IMU/body pose in the world frame after the bundle adjustment |
| `vina-slam-results/pcd/NNNNNN.pcd` | the motion-compensated points of frame `NNNNNN` (row `NNNNNN` of the trajectory) in its body frame |

The converter places each frame in the world with the pose of its row. The
frames take roughly 0.6 MB each for a Livox Mid-360. VINA-SLAM refuses to save
into an existing folder, so each run deletes the previous `vina-slam-results/`.

**Timestamps:** the benchmark does not change VINA-SLAM's clock (no
`use_sim_time`). Each trajectory row is stamped with its scan's end time taken
from the LiDAR data, i.e. bag time.

The RViz view is VINA-SLAM's own `rviz_cfg.rviz` with RViz's default panels
(Displays, Views, Time), see
[overlay/rviz/vina_slam_bench.rviz](overlay/rviz/vina_slam_bench.rviz).

## Contact

januszbedkowski@gmail.com
