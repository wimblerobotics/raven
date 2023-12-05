from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='serial_port_right_rear', 
            default_value='/dev/lidar_right_rear',
            description='LD06 Serial Port for right_rear device'
        ),
        DeclareLaunchArgument(
            name='serial_port_left_front', 
            default_value='/dev/lidar_left_front',
            description='LD06 Serial Port for left_front device'
        ),
        DeclareLaunchArgument(
            name='topic_name_right_rear', 
            default_value='scan_right_rear',
            description='LD06 Topic Name for right_rear frame'
        ),
        DeclareLaunchArgument(
            name='topic_name_left_front', 
            default_value='scan_left_front',
            description='LD06 Topic Name for left_front frame'
        ),
        DeclareLaunchArgument(
            name='lidar_frame_right_rear', 
            default_value='lidar_frame_right_rear',
            description='Lidar Frame ID for right_rear device'
        ),
        DeclareLaunchArgument(
            name='lidar_frame_left_front', 
            default_value='lidar_frame_left_front',
            description='Lidar Frame ID for left_front device'
        ),
        DeclareLaunchArgument(
            name='range_threshold', 
            default_value='0.0508',
            description='Range Threshold'
        ),
        # Node(
        #     package='ldlidar',
        #     executable='ldlidar',
        #     name='ldlidar_right_rear',
        #     output='screen',
        #     #prefix=['xterm -e gdb -ex run --args'],
        #     parameters=[
        #         {'serial_port': LaunchConfiguration("serial_port_right_rear")},
        #         {'topic_name': LaunchConfiguration("topic_name_right_rear")},
        #         {'lidar_frame': LaunchConfiguration("lidar_frame_right_rear")},
        #         {'range_threshold': LaunchConfiguration("range_threshold")}
        #     ]
        # ),
        Node(
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar_left_front',
            output='screen',
            #prefix=['xterm -e gdb -ex run --args'],
            parameters=[
                {'serial_port': LaunchConfiguration("serial_port_left_front")},
                {'topic_name': LaunchConfiguration("topic_name_left_front")},
                {'lidar_frame': LaunchConfiguration("lidar_frame_left_front")},
                {'range_threshold': LaunchConfiguration("range_threshold")}
            ]
        ),
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            parameters=[{
                    "destination_frame": "base_link",
                    "cloud_destination_topic": "/merged_lidar_cloud",
                    "scan_destination_topic": "/scan",
#                    "laserscan_topics": "/scan_left_front /scan_right_rear",
                    "laserscan_topics": "/scan_left_front",
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.013935472816228867,
                    "scan_time": 0.010,
                    "range_min": 0.0504,
                    "range_max": 20.0
            }],
            # prefix=['xterm -e gdb run -ex --args'],
            # respawn=True,
            output='screen')
    ])
