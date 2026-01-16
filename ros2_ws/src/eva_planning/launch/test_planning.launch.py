#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='circular_track'),
        DeclareLaunchArgument('vehicle_speed', default_value='3.0'),
        
        Node(
            package='eva_planning',
            executable='eva_test_generator.py',
            name='eva_test_generator',
            output='screen',
            parameters=[{
                'scenario': LaunchConfiguration('scenario'),
                'vehicle_speed': LaunchConfiguration('vehicle_speed'),
            }]
        ),
        
        Node(
            package='eva_planning',
            executable='eva_planning_node',
            name='eva_planning_node',
            output='screen'
        )
    ])
