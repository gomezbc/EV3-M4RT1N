#!/usr/bin/env python

import glob
from ament_index_python import get_package_share_directory
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
import numpy as np
import random as rd



class Robot(Node):
    def __init__(self):
        super().__init__('rwander_node')
        self.declare_parameter('vel_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('pose_topic', 'rosbot_pose')
        vel_topic_ = self.get_parameter('vel_topic').value
        scan_topic_ = self.get_parameter('scan_topic').value
        pose_topic_ = self.get_parameter('pose_topic').value

        # ROS Subscribers
        self._laser_sub = self.create_subscription(LaserScan, scan_topic_, self.obstacle_detect, 10)
        self._pose_sub = self.create_subscription(Pose2D, pose_topic_, self.pose_callback, 10)
        # ROS Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, vel_topic_, 10)       
        self._scan_count = 0
        self._max_range = 100.0
        self._uninitialized = 1
        self._bearings = []
        self._scan = []

        
        
    def obstacle_detect(self, scan_msg):
        
        self._scan = scan_msg.ranges

        if self._uninitialized:
            self._uninitialized = 0
            self._scan_count = len(scan_msg.ranges)
            self._max_range = scan_msg.range_max
            # Bearings stores the angle of each range measurement (radians)
            for i in range(0, self._scan_count):
                self._bearings.append(scan_msg.angle_min + scan_msg.angle_increment * i)
            self.get_logger().info("# Scan count %d"%(self._scan_count))  
            self.get_logger().info("# Laser angle min: %.2f"%(np.rad2deg(scan_msg.angle_min)))
            self.get_logger().info("# Laser angle max: %.2f"%(np.rad2deg(scan_msg.angle_max)))
            self.get_logger().info("# Laser angle increment:  %.4f rad (%.2f deg)"%(scan_msg.angle_increment, np.rad2deg(scan_msg.angle_increment)))
            self.get_logger().info("# Time between mesurements [seconds]:  %.2f"%(scan_msg.time_increment))
            self.get_logger().info("# Time between scans [seconds]:  %.2f"%(scan_msg.scan_time))
            self.get_logger().info("# Minimum range value:  %.2f"%(scan_msg.range_min))
            self.get_logger().info("# Maximum range value:  %.2f "%(scan_msg.range_max))
            resolution = (scan_msg.angle_max - scan_msg.angle_min)/len(scan_msg.ranges)
            self.get_logger().info("# Resolution:  %.2f"%(np.rad2deg(resolution))) 

        # Replace infinity values with max_range 
        self._scan = [x if x < self._max_range else self._max_range for x in self._scan] 

        # Reorganize scan indexes to make it easier to work with. 
        # 0 index corresponds to the back side of the robot for both, scan and bearings.
        self._scan = [self._scan[i - 800] for i in range(self._scan_count)]    
    
        # TODO: add your code here
        # IMPORTANT: valor pequeño cerca, grande lejos
        th = 3.5
        # con el th ignoramos los valores muy lejanos, es decir el ruido
        right = np.mean(list(filter(lambda x: x < th, self._scan[350:650])))
        front = np.mean(list(filter(lambda x: x < th, self._scan[650:950])))
        left = np.mean(list(filter(lambda x: x < th, self._scan[950:1250])))

        # if the values are nan set to th
        front = front if front else th
        left = left if left else th
        right = right if right else th

        right_abs = np.mean(self._scan[200:650])
        left_abs = np.mean(self._scan[950:1400])

        if front > 2.0:
            speed = 0.5
        elif front > 1.0:
            speed = 0.3
        else:
            speed = 0.1

        sides_dif = abs(left-right)

        # Use random turn angle to ensure it doesn't always follow same path
        # Two cases: near so left and right are close, far so left and right can also be close
        if front > 2.0:
            # Wall is close but not a lot
            turn_angle = rd.uniform(0.1, 0.3)
        else:
            # Wall is in front
            turn_angle = rd.uniform(0.5, 0.6)

        if left < right:
            turn = -turn_angle
        else:
            turn = turn_angle

        self.get_logger().info("Left %.2f | Front %.2f | Right %.2f | Speed %.2f | Turn %.2f"%(left,front,right, speed, turn))
        self.get_logger().info("Left_abs %.2f | Right_abs %.2f"%(left_abs,right_abs))
        ## end TODO
        cmd_vel_msg_ = Twist()
        cmd_vel_msg_.linear.x  = speed
        cmd_vel_msg_.linear.y  = 0.0
        cmd_vel_msg_.angular.z = turn
        self._cmd_vel_pub.publish( cmd_vel_msg_ ) 
    
    def pose_callback(self, msg):
        i = 1
        # lo he comentado porq ahora no lo necesitamos y ensucia el log
        #self.get_logger().info("Robot pose: x: %.2f, y: %.2f, theta: %.2f"%(msg.x, msg.y, msg.theta))
        

                                            
def main(args=None):
    rclpy.init(args=args)
    rwander_node = Robot()
    try:
        rclpy.spin(rwander_node)
    except KeyboardInterrupt:
        rwander_node.get_logger().info("Node interrupted by keyboard (CTRL+C)")
    finally:  
        rwander_node.destroy_node()  
        rclpy.shutdown()
    
if __name__=="__main__":
    main()
