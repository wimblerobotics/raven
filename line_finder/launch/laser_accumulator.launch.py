import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    do_standalone = True
    do_debug = True

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

    # debug
    # if (do_debug):
    #     debug_launch = ExecuteProcess(
    #         cmd=["/opt/ros/galactic/bin/ros2 run --prefix 'gdbserver localhost:3000' line_finder laser_accumulator"
    #         ],
    #         shell=True,
    #         output="screen"
    #     )
    #     ld.add_action(debug_launch)

    laser_accumulator_node = Node(
        # arguments=['--config', configFilePath],
        executable='laser_accumulator',
        name='laser_accumulator',
        package='line_finder',
        # prefix=['gdbserver localhost:3000 '],
        emulate_tty=True,
        # prefix=["'ros2 run --prefix gdbserver localhost:3000' line_finder laser_accumulator"],
        # prefix=["xterm -geometry 200x30 -e gdb -ex run --args"],
        output='screen'
    )

    ld.add_action(laser_accumulator_node)

    return ld
