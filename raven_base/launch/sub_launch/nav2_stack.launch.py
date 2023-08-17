from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription, launch_description
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
import os
import sys

raven_sub_launch_path = os.path.join(get_package_share_directory('raven_base'),
                                    'launch', 'sub_launch')
sys.path.append(raven_sub_launch_path)
# print(sys.path)
import common

def generate_launch_description():
  navigation_pkg_path = get_package_share_directory('raven_base')
  #navigation_pkg_path = get_package_share_directory('nav2_bringup')
  param_substitutions = {
      'default_nav_through_poses_bt_xml': os.path.join(navigation_pkg_path,"behavior_trees","navigate_through_poses_w_replanning_and_recovery.xml"),
      'default_nav_through_poses_bt_xml': os.path.join(navigation_pkg_path,"behavior_trees","navigate_through_poses_w_replanning_and_recovery.xml"),
      'default_nav_to_pose_bt_xml': os.path.join(navigation_pkg_path,"behavior_trees","navigate_to_pose_w_replanning_and_recovery.xml")
  }

  configured_params = RewrittenYaml(
          source_file=os.path.join(navigation_pkg_path,'config','nav2_params.yaml'),
        #   source_file=os.path.join(navigation_pkg_path,'params','nav2_params.yaml'),
          root_key='',
          param_rewrites=param_substitutions,
          convert_types=True)

  do_mapping = LaunchConfiguration('mapping')

  do_mapping_argument = DeclareLaunchArgument(
      'mapping',
      default_value='true',
      description='Enable mapping?')

  nav_pkg_path = get_package_share_directory('raven_base')

  use_sim = LaunchConfiguration('use_sim')

  nav_include = GroupAction(
      actions=[
          IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_pkg_path + '/launch/sub_launch/bringup_launch.py'),
                launch_arguments = {
                      'use_sim_time' : use_sim,
                      'map' : common.map_file_name,
                      'autostart' : 'true',
                      'slam' : do_mapping,
                      'params_file' : [configured_params]
                }.items(),

          )
      ]
  )

  return launch.LaunchDescription([nav_include])
