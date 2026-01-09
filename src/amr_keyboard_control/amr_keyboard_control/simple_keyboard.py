#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleKeyboard(Node):
    def __init__(self):
        super().__init__('simple_keyboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.2
        self.turn = 0.5
        self.get_logger().info("Enter: w/s/a/d/x")

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = input("Command (w/s/a/d/x): ").strip()

            if key == 'w':
                twist.linear.x = self.speed
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -self.speed
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = self.turn
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -self.turn
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.pub.publish(twist)

def main():
    rclpy.init()
    node = SimpleKeyboard()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
