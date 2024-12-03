#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist
from rclpy.qos import qos_profile_sensor_data
import numpy as np

SCAN_BUFFER = 10
CLOSE_RANGE = 0.1
CLOSE_MAX = 3

class Laser_Scan(Node):

    def __init__(self):
        super().__init__('laser_scan')

        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile=qos_profile_sensor_data)

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.close_counts = []

    def scan_callback(self, msg):
        
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        points = []

        for i, range in enumerate(ranges):
            range_angle = angle_min + (i * angle_increment)
            range_angle = self.normalize_angle(range_angle)

            close_count = 0

            point = Point()
            point.x = float(range * np.cos(range_angle))
            point.y = float(range * np.sin(range_angle))
            point.z = float(0)

            #prevent inf values in points
            if (point.x < 1000.0 and point.x > -1000.0 and point.y < 1000.0 and point.y > -1000.0):
                points.append(point)

            #if a point is less than .1m (~4 inches)
            if (range < CLOSE_RANGE):
                close_count += 1
            
        self.detect_objects(close_count)


    def detect_objects(self, count):
        #Only keeps data from most recent scans. The number of scans is determined by SCAN_BUFFER
        if(len(self.close_counts > SCAN_BUFFER)):
            self.close_counts.pop(0)
        self.close_counts.append(count)

        #if there are many scans with several close points, there is probably an object
        avg = np.average(self.close_counts)
        if (avg > CLOSE_MAX): 
            #stop the robot, then back up
            self.command_x_vel(0)
            time.sleep(0.5)
            self.command_x_vel(-0.25)
            time.sleep(0.5)
            self.command_x_vel(0)
            

    def command_x_vel(self, x_vel):
        #publish a message with all
        msg = Twist()
        msg.angular.x = x_vel
        self.cmd_vel_publisher.publish(msg)
        

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))      



def main(args=None):
    rclpy.init(args=args)

    node = Laser_Scan()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()