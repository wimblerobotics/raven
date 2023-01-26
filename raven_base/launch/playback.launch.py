from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import sys

raven_sub_launch_path = os.path.join(get_package_share_directory('raven_base'),
                                    'launch', 'sub_launch')
sys.path.append(raven_sub_launch_path)
# print(sys.path)
import common

def generate_launch_description():
    do_nav2 = False #os.getenv('DO_NAV2', default=True) in (
        # 'True', 'true', '1', 't', 'T')
    do_rviz = True #os.getenv('DO_RVIZ', default=True) in (
        # 'True', 'true', '1', 't', 'T')
    print(f'DO_NAV2: {do_nav2}')
    print(f'DO_RVIZ: {do_rviz}')

    # Bring up the robot description (URDF).
    raven_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            common.raven_description_directory_path, '/launch/raven_description.launch.py'
        ]))
    common.ld.add_action(raven_description_launch)

    # Bring up the Nav2 stack.
    if (do_nav2):
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                common.raven_base_directory_path,
                '/launch/sub_launch/nav2_stack.launch.py'
            ]), )
        common.ld.add_action(nav2_launch)
    
    # Playback
    play_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "--clock", 
            "--loop",
            "/home/ros/bag/bag_0.db3"
        ],
        output="screen"
    )
    common.ld.add_action(play_launch)

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
                                                'use_sim_time': False
                                            }])
        common.ld.add_action(rviz_node)

    # Launch
    return common.ld
