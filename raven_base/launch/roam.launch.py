from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros
import os
import sys

raven_sub_launch_path = os.path.join(get_package_share_directory('raven_base'),
                                    'launch', 'sub_launch')
sys.path.append(raven_sub_launch_path)
# print(sys.path)
import common

def generate_launch_description():
    do_joystick = os.getenv('DO_JOYSTICK',
                            default=False) in ('True', 'true', '1', 't', 'T')
    do_nav2 = os.getenv('DO_NAV2', default=True) in (
        'True', 'true', '1', 't', 'T')
    do_rviz = os.getenv('DO_RVIZ', default=True) in (
        'True', 'true', '1', 't', 'T')
    print(f'DO_JOYSTICK: {do_joystick}')
    print(f'DO_NAV2: {do_nav2}')
    print(f'DO_RVIZ: {do_rviz}')

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('raven_base'),
            'config',
            'nav2_params.yaml'))
    DeclareLaunchArgument(
        'map',
        default_value='map5',
        description='Full path to map file to load'),

    DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to param file to load'),

    # Bring up the T265 camera.
    t265_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [common.raven_base_directory_path, '/launch/sub_launch/t265.launch.py']))
    common.ld.add_action(t265_launch)

    # Bring up the joystick.
    if do_joystick:
        joystick_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                common.joystick_directory_path,
                '/launch/raven_bluetooth_joystick.launch.py'
            ]))
        common.ld.add_action(joystick_launch)

    # Bring up the twist multiplexer.
    multiplexer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [common.multiplexer_directory_path, '/launch/twist_multiplexer.launch.py'])
    )
    common.ld.add_action(multiplexer_launch)

    # Bring up the robot description (URDF).
    raven_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            common.raven_description_directory_path, '/launch/raven_description.launch.py'
        ]))
    common.ld.add_action(raven_description_launch)

    # Bring up the LIDARs.
    lidars_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [common.raven_base_directory_path, '/launch/sub_launch/ldlidars.launch.py']))
    common.ld.add_action(lidars_launch)
    
    # Bring up the OAK-Ds
    oakds_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [common.raven_base_directory_path, '/launch/sub_launch/oakds.launch.py']))
    common.ld.add_action(oakds_launch)

    # Bring up the Nav2 stack.
    if (do_nav2):
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                common.raven_base_directory_path,
                '/launch/sub_launch/bringup_launch.py'
            ]), 
            launch_arguments={
                'map': common.map_file_name,
                'use_sim_time': common.use_sim_time,
                'params_file': param_dir}.items()
            )
        common.ld.add_action(nav2_launch)

    # Bring up rviz2
    rviz_config_dir = os.path.join(
        common.rviz_directory_path, 'config', 'default_config.rviz')
    if do_rviz:
        rviz_node = launch_ros.actions.Node(package='rviz2',
                                            executable='rviz2',
                                            name='rviz2',
                                            output='screen',
                                            arguments=['-d', rviz_config_dir],
                                            parameters=[{
                                                'use_sim_time': common.use_sim_time
                                            }])
        common.ld.add_action(rviz_node)

    # Launch
    return common.ld
