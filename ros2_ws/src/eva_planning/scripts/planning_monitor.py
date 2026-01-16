#!/usr/bin/env python3
"""EVA Planning Monitor - Real-time statistics"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time
import math
from collections import deque

class PlanningMonitor(Node):
    def __init__(self):
        super().__init__('planning_monitor')
        
        self.create_subscription(Path, '/planning/global_path', self.global_callback, 10)
        self.create_subscription(Path, '/planning/local_path', self.local_callback, 10)
        self.create_subscription(Odometry, '/vehicle/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(String, '/planning/status', self.status_callback, 10)
        
        self.global_times = deque(maxlen=100)
        self.local_times = deque(maxlen=100)
        self.last_global_time = None
        self.last_local_time = None
        
        self.current_odom = None
        self.current_goal = None
        self.global_points = 0
        self.local_points = 0
        self.successes = 0
        self.failures = 0
        
        self.create_timer(2.0, self.display_stats)
        self.get_logger().info('Planning Monitor started')
    
    def global_callback(self, msg):
        now = time.time()
        if self.last_global_time:
            self.global_times.append(now - self.last_global_time)
        self.last_global_time = now
        self.global_points = len(msg.poses)
        self.successes += 1
    
    def local_callback(self, msg):
        now = time.time()
        if self.last_local_time:
            self.local_times.append(now - self.last_local_time)
        self.last_local_time = now
        self.local_points = len(msg.poses)
    
    def odom_callback(self, msg):
        self.current_odom = msg
    
    def goal_callback(self, msg):
        self.current_goal = msg
    
    def status_callback(self, msg):
        if 'FAILED' in msg.data:
            self.failures += 1
    
    def display_stats(self):
        print("\n" + "="*70)
        print("EVA PLANNING MONITOR")
        print("="*70)
        
        if self.current_odom:
            x = self.current_odom.pose.pose.position.x
            y = self.current_odom.pose.pose.position.y
            v = self.current_odom.twist.twist.linear.x
            print(f"\n📍 Vehicle: ({x:.2f}, {y:.2f}) m, {v:.2f} m/s")
        
        if self.current_goal:
            gx = self.current_goal.pose.position.x
            gy = self.current_goal.pose.position.y
            if self.current_odom:
                dist = math.sqrt((gx-x)**2 + (gy-y)**2)
                print(f"🎯 Goal: ({gx:.2f}, {gy:.2f}) m, dist: {dist:.2f} m")
        
        print(f"\n🌍 Global: ", end="")
        if self.global_times:
            rate = 1.0 / (sum(self.global_times) / len(self.global_times))
            print(f"{rate:.2f} Hz, {self.global_points} points")
        else:
            print("waiting...")
        
        print(f"📍 Local: ", end="")
        if self.local_times:
            rate = 1.0 / (sum(self.local_times) / len(self.local_times))
            print(f"{rate:.2f} Hz, {self.local_points} points")
        else:
            print("waiting...")
        
        total = self.successes + self.failures
        if total > 0:
            success_rate = 100.0 * self.successes / total
            print(f"\n📊 Success: {success_rate:.1f}% ({self.successes}/{total})")
        
        print("="*70)

def main(args=None):
    rclpy.init(args=args)
    monitor = PlanningMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
