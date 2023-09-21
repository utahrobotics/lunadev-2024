# Launches the current lunabot with development stuff as well
import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

sys.path.insert(1, get_package_share_directory('launchers') + "/launch")

import tele_tunnel_action


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='drive',
        #     executable='drive',
        #     name='drive'
        # ),
        Node(
            package='telemetry',
            executable='telemetry',
            name='telemetry'
        ),
        Node(
            package='camera',
            executable='compress',
            name='compress'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('launchers'), 'launch'
                ),
                '/rs_launch.py'
            ])
        ),
        tele_tunnel_action.StartTelemetryTunnel()
    ])
