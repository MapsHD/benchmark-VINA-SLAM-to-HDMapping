"""Benchmark launch for VINA-SLAM: the vina_slam node only, without RViz (the
run script opens RViz in its own tmux pane).

The parameter file is one of VINA-SLAM's own (the run script's default is
mid360_bench.yaml: the author's mid360.yaml with the local BA on). Only the
run-specific settings are passed on top of it: the bag VINA-SLAM reads by
itself, where it saves, and saving of the poses and per-frame point clouds
that the converter reads.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('config_file', description='VINA-SLAM parameter file (full path)'),
        DeclareLaunchArgument('bag_path', description='ROS 2 bag directory that VINA-SLAM reads'),
        DeclareLaunchArgument('save_path', description="folder, ending in '/', in which VINA-SLAM creates <bagname>/"),
        DeclareLaunchArgument('bagname', default_value='vina-slam-results',
                              description='name of the folder VINA-SLAM saves into; it must not exist yet'),
        Node(
            package='vina_slam',
            executable='vina_slam',
            name='vina_slam',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'General.bag_path': LaunchConfiguration('bag_path'),
                    'General.save_path': LaunchConfiguration('save_path'),
                    'General.bagname': LaunchConfiguration('bagname'),
                    # <save_path>/<bagname>/trajectory.txt and pcd/NNNNNN.pcd
                    'General.is_save_pose': 1,
                    'General.is_save_map': 1,
                    'General.pose_save_path': '',
                    'General.pose_filename': 'trajectory.txt',
                },
            ],
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
    ])
