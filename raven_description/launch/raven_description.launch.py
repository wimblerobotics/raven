import os
import sys
import xacro
import yaml

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_state_pub_gui')

    my_package_name = 'raven_description'
    pkg_share = get_package_share_directory(my_package_name)
    default_model_path = os.path.join(pkg_share, 'urdf', 'raven_wot265.urdf.xacro')

    joint_state_configFilePath = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'joint_state_publisher.yaml'
    )

    # with open(joint_state_configFilePath, 'r') as file:
    #     joint_state_configParams = yaml.safe_load(file)['joint_state_publisher']['ros__parameters']   

    # robot_description_raw = xacro.process_file(default_model_path).toxml()

    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description_raw,
    #                  'use_sim_time': use_sim_time
    #                  }]
    # )

    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[joint_state_configParams],
    #     condition=launch.conditions.UnlessCondition(use_gui)
    # )

    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(use_gui)
    # )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(name='use_state_pub_gui', default_value='False',
                                             description='Flag to enable joint_state_publisher_gui'),
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        # robot_state_publisher_node,
    ])
