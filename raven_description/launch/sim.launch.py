import os
import sys
import xacro
import yaml

import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True

    my_package_name = 'raven_description'
    pkg_share = get_package_share_directory(my_package_name)
    default_model_path = os.path.join(pkg_share, 'urdf', 'raven_wot265.urdf.xacro')

    joint_state_configFilePath = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'joint_state_publisher.yaml'
    )

    with open(joint_state_configFilePath, 'r') as file:
        joint_state_configParams = yaml.safe_load(file)['joint_state_publisher']['ros__parameters']   

    robot_description_raw = xacro.process_file(default_model_path).toxml()

    robot_state_publisher_node = launch_ros.actions.Node(
        executable='robot_state_publisher',
        output='screen',
        package='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': use_sim_time
                     }]
    )

    # lidar_merge_node = launch_ros.actions.Node(
    #         package='ira_laser_tools',
    #         executable='laserscan_multi_merger',
    #         name='laserscan_multi_merger',
    #         parameters=[{
    #                 "destination_frame": "base_link",
    #                 "cloud_destination_topic": "/merged_lidar_cloud",
    #                 "scan_destination_topic": "/scan",
    #                 "laserscan_topics": "/scan_left_front /scan_right_rear",
    #                 "angle_min": -3.14159,
    #                 "angle_max": 3.14159,
    #                 "angle_increment": 0.013935472816228867,
    #                 "scan_time": 0.010,
    #                 "range_min": 0.010,
    #                 "range_max": 50.0
    #         }],
    #         # prefix=['xterm -e gdb run -ex --args'],
    #         # respawn=True,
    #         output='screen')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={
              'world': os.path.join(pkg_share, 'worlds', 'test_world.world')
            }.items()
        )

    spawn_entity = launch_ros.actions.Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'my_bot'],
        output='screen')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(name='use_state_pub_gui', default_value='False',
                                             description='Flag to enable joint_state_publisher_gui'),
        gazebo,
        # lidar_merge_node,
        robot_state_publisher_node,
        spawn_entity
    ])
