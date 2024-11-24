#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
import math


class MotionModelNode(Node):
    """
    A ROS 2 Node for implementing the robot's motion model.
    This node subscribes to current state and control inputs, computes the next state,
    and publishes the updated state.
    """

    def __init__(self):
        super().__init__('motion_model_node')  # Initialize the node with a name

        # Declare parameters for robot properties
        self.declare_parameter('wheel_radius', 0.05)  # Radius of the wheels (default: 0.05 meters)
        self.declare_parameter('wheel_base', 0.2)  # Distance between the wheels (default: 0.2 meters)

        # Retrieve parameters from the ROS parameter server
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        # Set up subscribers to listen for state and control input messages
        self.state_sub = self.create_subscription(
            Pose2D,  # Message type: Pose2D (x, y, theta)
            'current_state',  # Topic name
            self.state_callback,  # Callback function to handle incoming messages
            10  # Queue size
        )
        self.control_sub = self.create_subscription(
            Float32MultiArray,  # Message type: Array of float values
            'control_inputs',  # Topic name
            self.control_callback,  # Callback function to handle incoming messages
            10  # Queue size
        )

        # Set up a publisher to send out the calculated next state
        self.state_pub = self.create_publisher(
            Pose2D,  # Message type: Pose2D (x, y, theta)
            'next_state',  # Topic name
            10  # Queue size
        )

        # Initialize variables to hold the robot's state and control inputs
        self.current_state = Pose2D()  # Default state (x=0, y=0, theta=0)
        self.control_inputs = None  # Control inputs will be set when received
        self.time_step = 0.1  # Time step for the motion model (default: 0.1 seconds)

        self.get_logger().info('Motion Model Node Initialized.')  # Log a startup message

    def state_callback(self, msg):
        """
        Callback to handle updates to the robot's current state.
        :param msg: Pose2D message containing the robot's position and orientation (x, y, theta).
        """
        self.current_state = msg  # Update the current state variable

    def control_callback(self, msg):
        """
        Callback to handle updates to the robot's control inputs.
        :param msg: Float32MultiArray containing two elements: [v_l, v_r].
        """
        # Ensure the control input contains exactly two elements
        if len(msg.data) != 2:
            self.get_logger().error("Control inputs must contain exactly 2 elements [v_l, v_r].")
            return

        # Update control inputs and calculate the next state
        self.control_inputs = msg.data
        self.calculate_and_publish_next_state()

    def calculate_and_publish_next_state(self):
        """
        Calculate the next state of the robot using the motion model and publish it.
        """
        # Check if control inputs are available
        if self.control_inputs is None:
            self.get_logger().warning("Control inputs are not yet received.")
            return

        # Extract the current state and control inputs
        x, y, theta = self.current_state.x, self.current_state.y, self.current_state.theta
        v_l, v_r = self.control_inputs  # Left and right wheel velocities

        # Compute linear and angular velocities
        v = self.wheel_radius * (v_r + v_l) / 2  # Linear velocity
        omega = self.wheel_radius * (v_r - v_l) / self.wheel_base  # Angular velocity

        # Update the robot's state based on motion model equations
        if omega == 0:  # If angular velocity is zero, the robot moves in a straight line
            new_x = x + v * math.cos(theta) * self.time_step
            new_y = y + v * math.sin(theta) * self.time_step
            new_theta = theta
        else:  # If angular velocity is non-zero, the robot moves along a curve
            new_x = x + (v / omega) * (math.sin(theta + omega * self.time_step) - math.sin(theta))
            new_y = y + (v / omega) * (math.cos(theta) - math.cos(theta + omega * self.time_step))
            new_theta = theta + omega * self.time_step

        # Normalize theta to keep it within the range [-π, π]
        new_theta = (new_theta + math.pi) % (2 * math.pi) - math.pi

        # Create a new Pose2D message for the updated state
        next_state = Pose2D()
        next_state.x = new_x
        next_state.y = new_y
        next_state.theta = new_theta

        # Publish the updated state
        self.state_pub.publish(next_state)

        # Log the updated state for debugging purposes
        self.get_logger().info(f"Published Next State: x={new_x:.2f}, y={new_y:.2f}, theta={new_theta:.2f}")


def main(args=None):
    """
    Entry point for the script. Initializes the node and starts spinning to process messages.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python library
    motion_model_node = MotionModelNode()  # Create an instance of the MotionModelNode

    try:
        rclpy.spin(motion_model_node)  # Keep the node running and processing callbacks
    except KeyboardInterrupt:
        pass  # Handle manual interruption gracefully
    finally:
        # Cleanup and shutdown
        motion_model_node.destroy_node()  # Destroy the node
        rclpy.shutdown()  # Shutdown the ROS 2 system


if __name__ == '__main__':
    main()
