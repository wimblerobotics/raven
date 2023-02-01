import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    do_standalone = True

    raven_description_directory_path = get_package_share_directory(
        'raven_description')

    configFilePath = os.path.join(
        get_package_share_directory('line_finder'),
        'config',
        'line_finder.yaml'
    )

    ld = LaunchDescription()

    if (do_standalone):
        # Bring up the robot description (URDF).
        raven_description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                raven_description_directory_path, '/launch/raven_description.launch.py'
            ]))
        
        ld.add_action(raven_description_launch)

    linecv_node = Node(
        arguments=['--config', configFilePath],
        executable='linecv',
        name='line_finder',
        package='line_finder',
        # prefix=["xterm -geometry 200x30 -e gdb -ex run --args"],
        output='screen'
    )

    ld.add_action(linecv_node)

    return ld
