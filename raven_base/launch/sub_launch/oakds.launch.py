import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('depthai_examples'),
                                'rviz', 'stereoPointCloud.rviz')
    urdf_launch_dir = os.path.join(
        get_package_share_directory('depthai_bridge'), 'launch')

    tf_prefix = LaunchConfiguration('tf_prefix',   default='oakd_left')

    mode = LaunchConfiguration('mode', default='depth')
    lrcheck = LaunchConfiguration('lrcheck', default=True)
    extended = LaunchConfiguration('extended', default=False)
    subpixel = LaunchConfiguration('subpixel', default=True)
    confidence = LaunchConfiguration('confidence', default=200)
    LRchecktresh = LaunchConfiguration('LRchecktresh', default=5)
    monoResolution = LaunchConfiguration('monoResolution',  default='720p')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='set to depth or disparity. Setting to depth will publish depth or else will publish disparity.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_monoResolution_cmd = DeclareLaunchArgument(
        'monoResolution',
        default_value=monoResolution,
        description='Contains the resolution of the Mono Cameras. Available resolutions are 800p, 720p & 400p for OAK-D & 480p for OAK-D-Lite.')

    stereo_node_left = launch_ros.actions.Node(
        package='depthai_examples',
        executable='stereo_node',
        name='stereo_node_left',
        namespace='oakd_left',
        output='screen',
        parameters=[{'tf_prefix': tf_prefix},
                    {'mode': mode},
                    {'lrcheck': lrcheck},
                    {'extended': extended},
                    {'subpixel': subpixel},
                    {'confidence': confidence},
                    {'LRchecktresh': LRchecktresh},
                    {'monoResolution': monoResolution}]
    )

    stereo_node_right = launch_ros.actions.Node(
        package='depthai_examples',
        executable='stereo_node',
        name='stereo_node_right',
        namespace='oakd_right',
        output='screen',
        parameters=[{'tf_prefix': tf_prefix},
                    {'mode': mode},
                    {'lrcheck': lrcheck},
                    {'extended': extended},
                    {'subpixel': subpixel},
                    {'confidence': confidence},
                    {'LRchecktresh': LRchecktresh},
                    {'monoResolution': monoResolution}]
    )

    metric_converter_node_left = launch_ros.actions.ComposableNodeContainer(
        name='container_converter_left',
        namespace='oakd_left',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node_left',
                    namespace='oakd_left',
                    remappings=[('image_raw', 'stereo/depth'),
                                ('camera_info', 'stereo/camera_info'),
                                ('image', 'stereo/converted_depth')]
                )
        ],
        output='screen'
    )

    metric_converter_node_right = launch_ros.actions.ComposableNodeContainer(
        name='container_converter_right',
        namespace='oakd_right',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    namespace='oakd_right',
                    remappings=[('image_raw', 'stereo/depth'),
                                ('camera_info', 'stereo/camera_info'),
                                ('image', 'stereo/converted_depth')]
                )
        ],
        output='screen'
    )

    point_cloud_node_left = launch_ros.actions.ComposableNodeContainer(
        name='container_point_cloud_left',
        namespace='oakd_left',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',
                    namespace='oakd_left',
                    remappings=[('depth/image_rect', 'stereo/converted_depth'),
                                ('intensity/image_rect', 'right/image_rect'),
                                ('intensity/camera_info', 'right/camera_info'),
                                ('points', 'stereo/points')]
                ),
        ],
        output='screen'
    )

    point_cloud_node_right = launch_ros.actions.ComposableNodeContainer(
        name='container_point_cloud_right',
        namespace='oakd_right',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',
                    namespace='oakd_right',
                    remappings=[('depth/image_rect', 'stereo/converted_depth'),
                                ('intensity/image_rect', 'right/image_rect'),
                                ('intensity/camera_info', 'right/camera_info'),
                                ('points', 'stereo/points')]
                ),
        ],
        output='screen'
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['--display-config', default_rviz])

    ld = LaunchDescription()
    ld.add_action(declare_tf_prefix_cmd)

    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)
    ld.add_action(declare_monoResolution_cmd)

    ld.add_action(stereo_node_left)
    # ld.add_action(stereo_node_right)

    ld.add_action(metric_converter_node_left)
    # ld.add_action(metric_converter_node_right)

    ld.add_action(point_cloud_node_left)
    # ld.add_action(point_cloud_node_right)

    ld.add_action(rviz_node)
    return ld
