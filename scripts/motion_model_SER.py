#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
import math

class MotionModelNode(Node):
    def __init__(self):
        super().__init__('motion_model_node')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.2)
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.state_sub = self.create_subscription(
            Pose2D,
            'current_state',
            self.state_callback,
            10
        )
        self.control_sub = self.create_subscription(
            Float32MultiArray,
            'control_inputs',
            self.control_callback,
            10
        )

        self.state_pub = self.create_publisher(
            Pose2D,
            'next_state',
            10
        )
        self.current_state = Pose2D()
        self.control_inputs = None
        self.time_step = 0.1

        self.get_logger().info('Motion Model Node Initialized.')
    def state_callback(self, msg):
        self.current_state = msg

    def control_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().error("Control inputs must contain exactly 2 elements [v_l, v_r].")
            return

        self.control_inputs = msg.data
        self.calculate_and_publish_next_state()

    def calculate_and_publish_next_state(self):
        if self.control_inputs is None:
            self.get_logger().warning("Control inputs are not yet received.")
            return
        x, y, theta = self.current_state.x, self.current_state.y, self.current_state.theta
        v_l, v_r = self.control_inputs
        
        v = self.wheel_radius * (v_r + v_l) / 2
        omega = self.wheel_radius * (v_r - v_l) / self.wheel_base

        if omega == 0:
            new_x = x + v * math.cos(theta) * self.time_step
            new_y = y + v * math.sin(theta) * self.time_step
            new_theta = theta
        else:
            new_x = x + (v / omega) * (math.sin(theta + omega * self.time_step) - math.sin(theta))
            new_y = y + (v / omega) * (math.cos(theta) - math.cos(theta + omega * self.time_step))
            new_theta = theta + omega * self.time_step

        new_theta = (new_theta + math.pi) % (2 * math.pi) - math.pi

        next_state = Pose2D()
        next_state.x = new_x
        next_state.y = new_y
        next_state.theta = new_theta

        self.state_pub.publish(next_state)
        self.get_logger().info(f"Published Next State: x={new_x:.2f}, y={new_y:.2f}, theta={new_theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    motion_model_node = MotionModelNode()

    try:
        rclpy.spin(motion_model_node)
    except KeyboardInterrupt:
        pass
    finally:
        motion_model_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
