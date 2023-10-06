# Launches the current lunabot
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    launchers_pkg_share = get_package_share_directory('launchers')
    robot_desc_pkg_share = get_package_share_directory('bot_descriptions')
    default_model_path = os.path.join(
        robot_desc_pkg_share,
        'src/description/lunabot_description.urdf'
    )

    return LaunchDescription([
        # Node(
        #     package='drive',
        #     executable='drive',
        #     name='drive'
        # ),
        # Declare udf
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        # Start URDF publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}
            ]
        ),
        # Start telemetry
        Node(
            package='telemetry',
            executable='telemetry',
            name='telemetry'
        ),
        # Start camera image compressor
        Node(
            package='camera',
            executable='compress',
            name='compress'
        ),
        # Start Pozyx
        Node(
            package='positioning',
            executable='positioning',
            name='positioning'
        ),
        # Start localization algorithm
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[
                os.path.join(launchers_pkg_share, 'config/ekf.yaml'),
                # {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        # Start rtabmap SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            remappings=[
                ('/scan_cloud', '/camera/depth/color/points'),
                ('/rgb/camera_info', '/camera/color/camera_info'),
                ('/rgb/image', '/camera/color/image_raw'),
            ],
            arguments=['--delete_db_on_start'],
            parameters=[
                {
                    "subscribe_scan_cloud": True,
                    "subscribe_depth": False,
                    "odom_frame_id": "odom",
                }
            ]
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            remappings=[
                ('/scan_cloud', '/camera/depth/color/points'),
                ('/rgb/camera_info', '/camera/color/camera_info'),
                ('/rgb/image', '/camera/color/image_raw'),
            ],
            parameters=[
                {
                    "subscribe_scan_cloud": True,
                    "odom_frame_id": "odom",
                }
            ]
        ),
        # Start RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    launchers_pkg_share, 'launch'
                ),
                '/rs_launch.py'
            ])
        ),
    ])