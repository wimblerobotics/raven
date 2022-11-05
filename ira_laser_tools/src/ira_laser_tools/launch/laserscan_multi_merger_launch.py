import os
import yaml

from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        parameters=[{
                "destination_frame": "base_link",
                "cloud_destination_topic": "/merged_lidar_cloud",
                "scan_destination_topic": "/merged_lidar_scan",
                "laserscan_topics": "/scan_left_front /scan_right_rear",
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.013935472816228867,
                "scan_time": 0.010,
                "range_min": 0.010,
                "range_max": 50.0
        }],
        #prefix=['xterm -e gdb run -ex --args'],
        # respawn=True,
        output='screen')
    return launch.LaunchDescription([merger_node])
