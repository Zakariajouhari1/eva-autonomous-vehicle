#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eva_planning',
            executable='eva_planning_node',
            name='eva_planning_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('eva_planning'),
                    'config', 'planning_params.yaml'
                ])
            ]
        )
    ])
