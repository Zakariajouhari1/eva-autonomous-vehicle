#!/usr/bin/env python3
"""EVA Test Data Generator - Simulates vehicle and environment"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class EVATestDataGenerator(Node):
    def __init__(self):
        super().__init__('eva_test_generator')
        
        self.declare_parameter('scenario', 'circular_track')
        self.declare_parameter('vehicle_speed', 3.0)
        
        self.scenario = self.get_parameter('scenario').value
        self.vehicle_speed = self.get_parameter('vehicle_speed').value
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/vehicle/odom', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # State
        self.vehicle_x = 0.0
        self.vehicle_y = -40.0
        self.vehicle_yaw = math.pi / 2
        self.time = 0.0
        
        # Setup scenario
        self.obstacles = [
            {'x': 30.0, 'y': 0.0, 'radius': 2.0},
            {'x': 0.0, 'y': 30.0, 'radius': 2.0},
            {'x': -30.0, 'y': 0.0, 'radius': 2.0},
            {'x': 0.0, 'y': -30.0, 'radius': 2.0},
        ]
        
        self.waypoints = []
        radius = 40.0
        for i in range(8):
            angle = 2 * math.pi * i / 8
            self.waypoints.append({'x': radius * math.cos(angle), 
                                   'y': radius * math.sin(angle)})
        
        self.current_wp_idx = 0
        
        # Timers
        self.create_timer(0.02, self.update_vehicle)  # 50 Hz
        self.create_timer(2.0, self.publish_map)
        self.create_timer(5.0, self.publish_next_goal)
        
        self.publish_map()
        self.get_logger().info(f'Test generator started: {self.scenario}')
    
    def update_vehicle(self):
        self.time += 0.02
        
        # Circular motion
        radius = 40.0
        angular_vel = self.vehicle_speed / radius
        
        self.vehicle_x = radius * math.cos(angular_vel * self.time)
        self.vehicle_y = radius * math.sin(angular_vel * self.time)
        self.vehicle_yaw = angular_vel * self.time + math.pi / 2
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.vehicle_x
        odom.pose.pose.position.y = self.vehicle_y
        
        cy = math.cos(self.vehicle_yaw * 0.5)
        sy = math.sin(self.vehicle_yaw * 0.5)
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy
        
        odom.twist.twist.linear.x = self.vehicle_speed
        
        self.odom_pub.publish(odom)
    
    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'
        
        resolution = 0.5
        width = 200
        height = 200
        
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = -50.0
        grid.info.origin.position.y = -50.0
        grid.info.origin.orientation.w = 1.0
        
        data = [0] * (width * height)
        
        for obs in self.obstacles:
            gx = int((obs['x'] + 50.0) / resolution)
            gy = int((obs['y'] + 50.0) / resolution)
            gr = int(obs['radius'] / resolution) + 1
            
            for dy in range(-gr, gr + 1):
                for dx in range(-gr, gr + 1):
                    if dx*dx + dy*dy <= gr*gr:
                        x = gx + dx
                        y = gy + dy
                        if 0 <= x < width and 0 <= y < height:
                            data[y * width + x] = 100
        
        grid.data = data
        self.map_pub.publish(grid)
        self.get_logger().info(f'Map published: {width}x{height}')
    
    def publish_next_goal(self):
        if not self.waypoints:
            return
        
        idx = (self.current_wp_idx + 1) % len(self.waypoints)
        wp = self.waypoints[idx]
        
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Goal: ({wp["x"]:.1f}, {wp["y"]:.1f})')

def main(args=None):
    rclpy.init(args=args)
    node = EVATestDataGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
