#!/usr/bin/env python3
"""Preflight check for the VINA-SLAM benchmark run.

VINA-SLAM reads the bag itself, only the LiDAR and IMU topics of its parameter
file (General.lid_topic and General.imu_topic), and decodes the LiDAR messages
by General.lidar_type. If a topic is missing from the bag, or has a type
VINA-SLAM does not decode for that lidar_type, it processes nothing and the
run saves no output, so the check stops the run first (exit code 42, the run
script then skips the conversion).

Every failure, an unexpected one included, ends in exit code 42 with a message:
starting a long run that cannot produce output is never better.

Usage: vina_slam_preflight.py <parameter_file.yaml> <ros2_bag_dir>
       vina_slam_preflight.py --topics <parameter_file.yaml>   (prints the input topics)
"""

import os
import sys

import yaml

PREFLIGHT_FAILED = 42

POINTCLOUD2 = "sensor_msgs/msg/PointCloud2"
IMU = "sensor_msgs/msg/Imu"
LIVOX_CUSTOM = "livox_ros_driver2/msg/CustomMsg"

# General.lidar_type as VINA-SLAM's config files list it. 0 is read as a Livox
# CustomMsg, every other type as a PointCloud2.
LIDAR_TYPES = {
    0: "Livox CustomMsg",
    1: "Velodyne",
    2: "Ouster",
    3: "Hesai",
    4: "RoboSense",
    5: "Simulation",
    6: "RoboSense Airy",
    7: "Livox MID360 PointCloud2",
}


class PreflightError(Exception):
    """An input problem, reported as it is."""


def fail(message):
    # stderr: in --topics mode the run script captures stdout as the topic list.
    print(f"[preflight] ERROR: {message}", file=sys.stderr)
    sys.exit(PREFLIGHT_FAILED)


def read_config(config_path):
    """Returns (lid_topic, imu_topic, lidar_type) from a VINA-SLAM parameter file."""
    try:
        with open(config_path) as f:
            config = yaml.safe_load(f)
    except OSError as e:
        raise PreflightError(f"cannot open the parameter file {config_path}: {e}") from e
    except yaml.YAMLError as e:
        raise PreflightError(f"the parameter file {config_path} is not valid YAML: {e}") from e

    try:
        general = config["vina_slam"]["ros__parameters"]["General"]
        lid_topic = general["lid_topic"]
        imu_topic = general["imu_topic"]
        lidar_type = general["lidar_type"]
    except (KeyError, TypeError) as e:
        raise PreflightError(
            f"{config_path} lacks vina_slam/ros__parameters/General/{{lid_topic, imu_topic, lidar_type}} "
            f"(missing: {e})") from e

    if not isinstance(lid_topic, str) or not isinstance(imu_topic, str):
        raise PreflightError(f"lid_topic and imu_topic in {config_path} must be topic names")
    if isinstance(lidar_type, bool) or not isinstance(lidar_type, int) or lidar_type not in LIDAR_TYPES:
        known = ", ".join(f"{k}={v}" for k, v in LIDAR_TYPES.items())
        raise PreflightError(f"unknown lidar_type {lidar_type!r} in {config_path} (known: {known})")
    return lid_topic, imu_topic, lidar_type


def read_bag_topics(bag_dir):
    """Returns {topic: (type, message_count)} from a ROS 2 bag's metadata.yaml."""
    metadata_path = os.path.join(bag_dir, "metadata.yaml")
    try:
        with open(metadata_path) as f:
            info = yaml.safe_load(f)["rosbag2_bagfile_information"]
        return {t["topic_metadata"]["name"]: (t["topic_metadata"]["type"], t.get("message_count", 0))
                for t in info["topics_with_message_count"]}
    except OSError as e:
        raise PreflightError(f"cannot read the ROS 2 bag metadata {metadata_path}: {e}") from e
    except (KeyError, TypeError, AttributeError, yaml.YAMLError) as e:
        raise PreflightError(f"unexpected content in {metadata_path}: {type(e).__name__}: {e}") from e


def check(config_path, bag_dir):
    lid_topic, imu_topic, lidar_type = read_config(config_path)
    print(f"[preflight] parameter file: {config_path}")
    print(f"[preflight] lidar_type {lidar_type} ({LIDAR_TYPES[lidar_type]})")

    topics = read_bag_topics(bag_dir)
    lidar_msg = LIVOX_CUSTOM if lidar_type == 0 else POINTCLOUD2
    expected = [(lid_topic, lidar_msg), (imu_topic, IMU)]

    ok = True
    for name, msg_type in expected:
        found = topics.get(name)
        if found is None or found[0] != msg_type:
            print(f"[preflight] ERROR: {name} must be {msg_type} ({found[0] if found else 'not in the bag'}).")
            ok = False
        elif not found[1]:
            print(f"[preflight] ERROR: {name} has no messages in the bag.")
            ok = False
        else:
            print(f"[preflight] {name}: {found[1]} msgs ({msg_type})")

    if not ok:
        for msg_type in sorted({m for _, m in expected}):
            names = [t for t, (ty, _) in topics.items() if ty == msg_type]
            print(f"[preflight] {msg_type} topics in this bag: {', '.join(names) or '(none)'}")
        print("[preflight] Choose the parameter file of your sensor (CONFIG_FILE) or set "
              "lid_topic, imu_topic and lidar_type in it.")
        sys.exit(PREFLIGHT_FAILED)


def main():
    args = sys.argv[1:]
    if len(args) == 2 and args[0] == "--topics":
        lid_topic, imu_topic, _ = read_config(args[1])
        print(f"{lid_topic} {imu_topic}")
    elif len(args) == 2:
        check(args[0], args[1])
    else:
        print(__doc__)
        sys.exit(PREFLIGHT_FAILED)


if __name__ == "__main__":
    try:
        main()
    except PreflightError as e:
        fail(str(e))
    except Exception as e:  # a bug in this check must not start a run blind
        fail(f"the preflight check itself failed ({type(e).__name__}: {e})")
