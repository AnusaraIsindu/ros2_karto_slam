import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_karto',
            executable='slam_karto',
            name='slam_karto',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                
                # Force disable loop closure
                'do_loop_closing': False,
                
                # Frame configuration
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'shell_link',
                'throttle_scans': 1,
                'resolution': 0.05,
                'map_update_interval': 5.0,
                'transform_publish_period': 0.1,
                'transform_tolerance': 1.0,
                
                # SLAM tuning parameters
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_travel_distance': 0.3,
                'minimum_travel_heading': 0.261,
                'scan_buffer_size': 50,
                'scan_buffer_maximum_scan_distance': 15.0,
                'link_match_minimum_response_fine': 0.6,
                'link_scan_maximum_distance': 8.0,
            }],
            remappings=[
                ('scan', '/scan'),
            ]
        )
    ])
