#!/usr/bin/env python3
"""
Simulateur de véhicule simple pour EVA
Publie odométrie avec suivi de trajectoire
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math

class VehicleSimulator(Node):
    def __init__(self):
        super().__init__('vehicle_simulator')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/vehicle/odom', 10)
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/planning/global_path', self.path_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # État du véhicule
        self.x = 0.0
        self.y = 0.0
        self.speed = 0.0  # m/s
        self.target_speed = 15.0  # ~54 km/h
        
        # Trajectoire
        self.path = []
        self.current_waypoint_idx = 0
        self.is_following = False
        
        # Timer pour mise à jour (50 Hz)
        self.timer = self.create_timer(0.02, self.update)
        
        self.get_logger().info('Vehicle Simulator started')
        self.get_logger().info('Publishing odometry at /vehicle/odom')
    
    def goal_callback(self, msg):
        """Nouveau goal reçu"""
        self.get_logger().info(
            f'New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    
    def path_callback(self, msg):
        """Nouvelle trajectoire reçue"""
        if len(msg.poses) == 0:
            return
        
        self.get_logger().info(f'New path received: {len(msg.poses)} waypoints')
        
        # Stocker les waypoints
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_waypoint_idx = 0
        self.is_following = True
        
        # Démarrer depuis le premier point
        if self.path:
            self.x, self.y = self.path[0]
            self.get_logger().info(f'Starting from ({self.x:.2f}, {self.y:.2f})')
    
    def update(self):
        """Mise à jour de la simulation"""
        
        # Accélérer/décélérer progressivement
        if self.is_following and self.path:
            # Accélération
            if self.speed < self.target_speed:
                self.speed += 0.5  # m/s²
            elif self.speed > self.target_speed:
                self.speed -= 0.5
        else:
            # Décélération si pas de trajectoire
            if self.speed > 0:
                self.speed -= 1.0
            else:
                self.speed = 0.0
        
        # Limiter la vitesse
        self.speed = max(0.0, min(self.speed, self.target_speed))
        
        # Suivre la trajectoire
        if self.is_following and self.path and self.current_waypoint_idx < len(self.path):
            target_x, target_y = self.path[self.current_waypoint_idx]
            
            # Distance au waypoint
            dx = target_x - self.x
            dy = target_y - self.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < 2.0:  # Waypoint atteint (2m de tolérance)
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx >= len(self.path):
                    # Trajectoire terminée
                    self.is_following = False
                    self.speed = 0.0
                    self.get_logger().info('Destination reached!')
                else:
                    remaining = len(self.path) - self.current_waypoint_idx
                    self.get_logger().info(
                        f'Waypoint reached. {remaining} waypoints remaining')
            else:
                # Se déplacer vers le waypoint
                move_dist = self.speed * 0.02  # distance = vitesse * dt
                if move_dist > 0:
                    self.x += (dx / dist) * move_dist
                    self.y += (dy / dist) * move_dist
        
        # Publier odométrie
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        
        # Vitesse
        if self.is_following and self.path and self.current_waypoint_idx < len(self.path):
            target_x, target_y = self.path[self.current_waypoint_idx]
            dx = target_x - self.x
            dy = target_y - self.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > 0:
                odom.twist.twist.linear.x = (dx / dist) * self.speed
                odom.twist.twist.linear.y = (dy / dist) * self.speed
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
