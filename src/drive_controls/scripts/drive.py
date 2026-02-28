#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from rclpy.executors import MultiThreadedExecutor
import math


class Drive(Node):
    def __init__(self):
        super().__init__('drive_node')

        self.deadzone = 0.1   # for joystick floats

        # Store joystick values
        self.linear = 0.0
        self.rotational = 0.0
        self.speed = 0.0

        # Subscriber
        self.subscription = self.create_subscription(
            Joy,
            '/joy0',
            self.callback,
            10
        )

        # Publisher
        self.values = Int32MultiArray()
        self.publisher = self.create_publisher(
            Int32MultiArray,
            '/rover',
            10
        )

        

    def callback(self, msg):
        self.linear = msg.axes[1]
        self.rotational = msg.axes[0]
        self.speed = msg.axes[2]  # assume trigger 0→1

        self.move()

    def move(self):

        # Apply deadzone
        if abs(self.linear) < self.deadzone:
            self.linear = 0.0

        if abs(self.rotational) < self.deadzone:
            self.rotational = 0.0

        # Scale speed to 0–255
        speed_scaled = int((self.speed + 1) / 2 * 255)

        # Differential drive logic
        left = (self.linear - self.rotational) * speed_scaled
        right = (self.linear + self.rotational) * speed_scaled

        left = int(left)
        right = int(right)

        wheel_speeds = [
            right, left,
            right, left,
            right, left
        ]

        self.values.data = wheel_speeds
        self.publisher.publish(self.values)

        self.get_logger().info(f"Wheel speeds: {wheel_speeds}")

def main(args=None):
    rclpy.init(args=args)
    node = Drive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
