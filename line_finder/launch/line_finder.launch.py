import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():

    ld = LaunchDescription()
    line_finder_node = Node(
        # prefix=['xterm -e gdb -ex run --args'],
        package = 'line_finder',
        executable = 'lf',
        name = 'line_finder',
        output = 'screen'
    )

    ld.add_action(line_finder_node)

    return ld