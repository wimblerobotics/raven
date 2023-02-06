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
    default_model_path = os.path.join(pkg_share, 'urdf', 'raven.urdf.xacro')

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
        robot_state_publisher_node,
        spawn_entity
    ])
