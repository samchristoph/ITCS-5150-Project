#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from itcs_5150_project.msg import LineMsg, CornerMsg, DepthFeatures
import math

# ros2 interface list

#entry_points={
   # 'console_scripts': [
  #      'observation_model_node = observation_model.observation_model_node:main',
 #   ],
#}
#should add the above to setup.py 

class Line:

    def __init__(self, A, B, C, p_a, p_b, ID):
        self.A = A
        self.B = B
        self.C = C
        self.p_a = p_a
        self.p_b = p_b
        self.ID = ID
        self.length = self.compute_length(p_a, p_b)
        self.slope = self.compute_slope(A, B, C)

        self.msg = LineMsg()
        self.msg.A = A
        self.msg.B = B
        self.msg.C = C
        self.msg.p_a = p_a
        self.msg.p_b = p_b
        self.msg.ID = ID
        self.msg.length = self.length
        self.msg.slope = self.slope

    def compute_length(self, p_a, p_b):
        return math.sqrt((p_a.x - p_b.x)**2 + (p_a.y - p_b.y)**2)

    def compute_slope(self, A, B, C):
        return -A / B if B != 0 else float('inf')


class Corner:

    def __init__(self, ID, p_a, p_b, psi=0):
        self.ID = ID
        self.p_a = p_a
        self.p_b = p_b
        self.psi = psi

        self.msg = CornerMsg()
        self.msg.ID = ID
        self.msg.p_a = p_a
        self.msg.p_b = p_b
        self.msg.psi = psi


class ObservationModelNode(Node):

    def __init__(self):
        super().__init__('observation_model_node')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.features_pub = self.create_publisher(DepthFeatures, '/depth_features', 10)

        self.get_logger().info("Observation Model Node initialized.")

    def scan_callback(self, scan_msg):

        points = self.convert_scan_to_cartesian(scan_msg)
        lines = self.get_all_lines(points)
        corners = self.get_corners_from_lines(lines)

        self.visualize_lines(lines)
        self.visualize_corners(corners)

        depth_features = DepthFeatures()
        depth_features.lines = [line.msg for line in lines]
        depth_features.corners = [corner.msg for corner in corners]
        self.features_pub.publish(depth_features)

    def convert_scan_to_cartesian(self, scan_msg):

        points = []
        angle = scan_msg.angle_min
        for distance in scan_msg.ranges:
            if distance < scan_msg.range_max:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                points.append(Point(x=x, y=y, z=0.0))
            angle += scan_msg.angle_increment
        return points

    def get_all_lines(self, points):

        if len(points) < 2:
            return []

        p_a = points[0]
        p_b = points[-1]
        line = Line(A=0, B=1, C=-1, p_a=p_a, p_b=p_b, ID=1)
        return [line]

    def get_corners_from_lines(self, lines):

        if len(lines) < 2:
            return []

        p_a = lines[0].p_a
        p_b = lines[0].p_b
        corner = Corner(ID=1, p_a=p_a, p_b=p_b)
        return [corner]

    def visualize_lines(self, lines):
        marker = Marker()
        marker.header.frame_id = "laser_link"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.a = 1.0

        for line in lines:
            marker.points.append(line.p_a)
            marker.points.append(line.p_b)

        self.marker_pub.publish(marker)

    def visualize_corners(self, corners):
 
        marker = Marker()
        marker.header.frame_id = "laser_link"
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.b = 1.0
        marker.color.a = 1.0

        for corner in corners:
            marker.points.append(corner.p_a)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ObservationModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
