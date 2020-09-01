#!/usr/bin/env python

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    sensor_ip = LaunchConfiguration("sensor_ip", default="172.29.29.2")
    host_ip = LaunchConfiguration("host_ip", default="172.29.29.1")
    host_udp_port = LaunchConfiguration("host_udp_port", default="0")
    frame_id = LaunchConfiguration("frame_id", default="top_lidar_sensor_frame")
    skip = LaunchConfiguration("skip", default="0")
    angle_start = LaunchConfiguration("angle_start", default="0.0")
    angle_end = LaunchConfiguration("angle_end", default="0.0")
    time_offset = LaunchConfiguration("time_offset", default="0.0")
    channel_enabled = LaunchConfiguration("channel_enabled", default="true")
    general_system_state = LaunchConfiguration("general_system_state", default="true")
    derived_settings = LaunchConfiguration("derived_settings", default="true")
    measurement_data = LaunchConfiguration("measurement_data", default="true")
    intrusion_data = LaunchConfiguration("intrusion_data", default="true")
    application_io_data = LaunchConfiguration("application_io_data", default="true")
    use_persistent_config = LaunchConfiguration("use_persistent_config", default="false")

    return LaunchDescription(
        [
            Node(
                package="sick_safetyscanners",
                executable="sick_safetyscanners_node",
                name="sick_safetyscanners",
                output="screen",
                emulate_tty=True,
                remappings=[
                    ('/diagnostics', 'diagnostics')
                ],
                parameters=[
                    {
                        "sensor_ip": sensor_ip,
                        "host_ip": host_ip,
                        "host_udp_port": host_udp_port,
                        "frame_id": frame_id,
                        "skip": skip,
                        "angle_start": angle_start,
                        "angle_end": angle_end,
                        "time_offset": time_offset,
                        "channel_enabled": channel_enabled,
                        "general_system_state": general_system_state,
                        "derived_settings": derived_settings,
                        "measurement_data": measurement_data,
                        "intrusion_data": intrusion_data,
                        "application_io_data": application_io_data,
                        "use_persistent_config": use_persistent_config,
                    }
                ],
            ),
        ]
    )
