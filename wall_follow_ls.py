#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
                ("odom_topic", "odom"),
                ("output_filename", "wf_ls"),
            ],
        )
        self.kp = self.get_parameter("kp").value

        # velocity topic
        self.vel_topic = self.get_parameter("vel_topic").value
        # odometry topic
        self.odom_topic = self.get_parameter("odom_topic").value

        """ output filename (with no extension).
        kp value and .csv extension will be added
        to this name afterwards
        """
        f = self.get_parameter("output_filename").value
        fout = f + "_" + str(self.kp) + ".csv"
        file = open(fout, "w")
        self.csvwriter = csv.writer(file, delimiter=",")
        self.csvwriter.writerow(["kp", "error", "v", "w", "x", "y"])

        self.laser_sub = self.create_subscription(
            LaserScan, "scan", self.follow_wall, 1
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.get_position, 1
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
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
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
        # 1.- Kalkulatu irakurketa "motzen" proiekzioak / Compute the projection of short readings
        xpos = np.empty((self.scan_count, 1), float)
        ypos = np.empty((self.scan_count, 1), float)

        # regresioa kalkulatzeko erabiliko ditugun irakurketak
        j = 80

        # Aukeratu "irakurketa motzak" eta kalkulatu dagozkien puntuak / Select the short readings and calculate the corresponding points
        for i in range(800, 1200):
            xpos[i] = self.scan[i] * np.cos(self.bearings[i])
            ypos[i] = self.scan[i] * np.sin(self.bearings[i])
        
        """
        for i in range(800, 1200):
            if np.isnan(ypos[i]) or np.isinf(ypos[i]) or np.isnan(xpos[i]) or np.isinf(xpos[i]):
                self.get_logger().info("xpos[%d], ypos[%d]: %s, %s" % (i, i, str(xpos[i]), str(ypos[i])))
        """
        filtered_ypos = np.empty((self.scan_count, 1), float)
        filtered_xpos = np.empty((self.scan_count, 1), float)
        y = 0
        th = 3.0
        for i in range(800, 1200):
            # si la diferencia de distancia hacia delante esta dentro del th o los valores no son nullos, es una posicion filtrada
            if (xpos[i] < th) and (not(np.isnan(ypos[i])) and not(np.isinf(ypos[i])) and not(np.isnan(xpos[i])) and not(np.isinf(xpos[i]))):
                filtered_xpos[y] = xpos[i]
                filtered_ypos[y] = ypos[i]
                y += 1

        # Darle la vuelta para que hagamos resize desde la derecha para alante
        i2 = y-1
        for i1 in range(y):
            xpos[i1] = filtered_xpos[i2]
            ypos[i1] = filtered_ypos[i2]
            i2 -= 1

        # xpos = np.flip(filtered_xpos.copy(), axis=0)
        # ypos = np.flip(filtered_ypos.copy(), axis=0)

        # ensure that the array is contiguous in memory before resizing
        # xpos = np.ascontiguousarray(xpos)
        # ypos = np.ascontiguousarray(ypos)

        # 2.- Erregresio lineala: hautatutako puntuek irudikatzen duten zuzenaren ezaugarriak / Linear regression: compute the line from the selected points
        c1 = 0.0
        c0 = 0.0
        theta = 0.0

        # comprobamos que la j no sea mayor al numero de elementos que tenemos
        self.get_logger().info("j: %.2f y: %.2f" % (j,y))
        # j = min(j, y)
        # Como tenemos el th, nos interesa calcularlo con todos los valores
        j = y
        """
        for i in range(j):
            self.get_logger().info("xpos[%d], ypos[%d]: %s, %s" % (i, i, str(xpos[i]), str(ypos[i])))
        """
        if j > 5:
            # KONTUZ!! j indizeak regresioa kalkulatzeko erabiliko den puntu kopurua adierazten du!!
            # NOTE!! The j index indicates the number of points used in the regression!!
            xpos.resize(j, 1)
            ypos.resize(j, 1)

            model = LinearRegression()
            model.fit(xpos, ypos)
            c0 = float(model.intercept_)
            c1 = float(model.coef_)  # c1 represents the slope (m) of the fitted line y = mx + b
            # Kalkulatu robota eta paretaren arteko angelua / Calculate the angle between the robot and the wall
            self.get_logger().info(
                "Malda: %.2f Angelua: %.2f (%.2f degrees)"
                % (c1, theta, m.degrees(theta))
            )

            theta = np.arctan(c1)
            # 3.- Abiadurak finkatu / Set velocities
            w = - self.kp * theta
            v = 1.0
            self.get_logger().info("Abiadurak: v = %.2f w = %.2f" % (v, w))
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


def main(args=None):
    rclpy.init(args=args)
    wf_node = Robot()
    try:
        rclpy.spin(wf_node)
    except KeyboardInterrupt:
        wf_node.destroy_node()


if __name__ == "__main__":
    main()
