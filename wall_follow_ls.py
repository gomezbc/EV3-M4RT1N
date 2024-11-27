#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
import os
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math as m
from tf_transformations import euler_from_quaternion

import csv
from sklearn.linear_model import LinearRegression
import time
from rclpy.signals import SignalHandlerOptions


class Robot(Node):
    def __init__(self):
        super().__init__("wall_following_node")
        # kp value to use in the execution
        self.declare_parameters(
            namespace="",
            parameters=[
                ("kp", 1.3),
                ("vel_topic", "cmd_vel"),
                ("pose_topic", "/gz_pose"),
                ("output_filename", "wf_ls"),
            ],
        )
        self.kp = self.get_parameter("kp").value

        # velocity topic
        self.vel_topic = self.get_parameter("vel_topic").value
        # odometry topic
        self.pose_topic = self.get_parameter('pose_topic').value

        """ output filename (with no extension).
        kp value and .csv extension will be added
        to this name afterwards
        """
        f = self.get_parameter("output_filename").value
        fout = f + "_" + str(self.kp) + ".csv"
        self.file = open(fout, "w")
        self.file.write("funciono")
        self.csvwriter = csv.writer(self.file, delimiter=",")
        self.csvwriter.writerow(["kp", "error", "v", "w", "x", "y"])

        self.laser_sub = self.create_subscription(
            LaserScan, "scan", self.follow_wall, 1
        )
        self.pose_sub = self.create_subscription(
            Pose, self.pose_topic, self.get_position, 1
        )
        self.vel_pub = self.create_publisher(Twist, self.vel_topic, 1)

        self.scan_count = 0
        self.range_min = 0.0
        self.max_range = 8.0
        self.bearings = []
        self.rx = 0
        self.ry = 0
        self.rtheta = 0
        self.uninitialized = True

    def get_position(self, msg):
        # Gets robot pose (x, y, yaw) from odometry
        self.rx = msg.position.x
        self.ry = msg.position.y
        quat = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.rtheta = yaw

    def follow_wall(self, scan):
        lscan = scan.ranges
        cmd_vel_msg = Twist()
        if self.uninitialized:
            self.uninitialized = False
            self.scan_count = len(scan.ranges)
            self.range_min = scan.range_min
            for i in range(self.scan_count):
                self.bearings.append(scan.angle_min + scan.angle_increment * i)

        self.scan = [scan.ranges[i - 800] for i in range(self.scan_count)]
        # TODO Add code here
        # 1.- Kalkulatu irakurketa "motzen" proiekzioak
        xpos = np.empty((self.scan_count, 1), float)
        ypos = np.empty((self.scan_count, 1), float)

        # Aukeratu irakurketa motzak eta kalkulatu dagozkien puntuak
        for i in range(800, 1200):
            xpos[i] = self.scan[i] * np.cos(self.bearings[i])
            ypos[i] = self.scan[i] * np.sin(self.bearings[i])

        # Filtratu datu baliogabeak eta gorde egokiak
        filtered_ypos = np.empty((self.scan_count, 1), float)
        filtered_xpos = np.empty((self.scan_count, 1), float)
        y = 0
        th = 3.0  # Threshold-a irakurketak aukeratzeko
        for i in range(800, 1200):
            if (xpos[i] < th) and (not(np.isnan(ypos[i])) and not(np.isinf(ypos[i])) and not(np.isnan(xpos[i])) and not(np.isinf(xpos[i]))):
                filtered_xpos[y] = xpos[i]
                filtered_ypos[y] = ypos[i]
                y += 1

        # Aldatu ordena irakurketak egokitzeko
        i2 = y - 1
        for i1 in range(y):
            xpos[i1] = filtered_xpos[i2]
            ypos[i1] = filtered_ypos[i2]
            i2 -= 1

        # 2.- Erregresio lineala kalkulatu
        c1 = 0.0  # Malda
        c0 = 0.0  # Ebakidura
        theta = 0.0  # Angelua

        # Puntu nahiko badira, erregresioa kalkulatu
        j = y
        if j > 5:
            xpos.resize(j, 1)
            ypos.resize(j, 1)

            model = LinearRegression()
            model.fit(xpos, ypos)
            c0 = float(model.intercept_)
            c1 = float(model.coef_) # c1 represents the slope (m) of the fitted line y = mx + b
            # Kalkulatu robota eta paretaren arteko angelua / Calculate the angle between the robot and the wall
            theta = np.arctan(c1)  # Angelua kalkulatu
            # Abiadura angelarrean minus jarri dugu bestela paretaren kontra biratzen zelako
            w = - self.kp * theta  # Angular velocity
            v = 1.0  # Linear velocity
            self.get_logger().info("Abiadurak: v = %.2f w = %.2f" % (v, w))
            self.get_logger().info("x: %.2f, y = %.2f" % (self.rx, self.ry))

            cmd_vel_msg.linear.x = v
            cmd_vel_msg.angular.z = w

        else:
            # Puntu nahiko ez badaude
            self.get_logger().info("Not enough points for applying the linear regression")
            cmd_vel_msg.linear.x = 0.2
            cmd_vel_msg.angular.z = 0.0

        # Datuak CSV-ra idatzi
        self.csvwriter.writerow(
            [
                f"{self.kp:.2f}",
                f"{c0:.2f}",
                f"{c1:.2f}",
                f"{theta:.2f}",
                f"{cmd_vel_msg.linear.x:.2f}",
                f"{cmd_vel_msg.angular.z:.2f}",
                f"{self.rx:.2f}",
                f"{self.ry:.2f}",
            ]
        )
        self.vel_pub.publish(cmd_vel_msg)
        # END TODO

            cmd_vel_msg.linear.x = v
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = w

        else:
            self.get_logger().info(
                "Not enough points for applying the linear regression"
            )
            cmd_vel_msg.linear.x = 0.2
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = 0.0

        self.csvwriter.writerow(
            [
                f"{self.kp:.2f}",
                f"{c0:.2f}",
                f"{c1:.2f}",
                f"{theta:.2f}",
                f"{cmd_vel_msg.linear.x:.2f}",
                f"{cmd_vel_msg.angular.z:.2f}",
                f"{self.rx:.2f}",
                f"{self.ry:.2f}",
            ]
        )
        self.vel_pub.publish(cmd_vel_msg)

        def __del__(self):
            if self.file:
                self.file.close()


def main(args=None):
    rclpy.init(args=args)
    wf_node = Robot()
    try:
        rclpy.spin(wf_node)
    except KeyboardInterrupt:
        wf_node.destroy_node()


if __name__ == "__main__":
    main()
