#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import serial
import math

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
        self.get_logger().info("Connected to ESP32")

        self.timer = self.create_timer(0.02, self.read_serial)

    def cmd_cb(self, msg):
        cmd = "CMD,S\n"
        if msg.linear.x > 0.1:
            cmd = "CMD,F\n"
        elif msg.linear.x < -0.1:
            cmd = "CMD,B\n"
        elif msg.angular.z > 0.1:
            cmd = "CMD,L\n"
        elif msg.angular.z < -0.1:
            cmd = "CMD,R\n"

        self.ser.write(cmd.encode())

    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line.startswith("ODOM"):
                return

            _, x, y, yaw, v, w = line.split(',')
            x, y, yaw = float(x), float(y), float(yaw)
            v, w = float(v), float(w)

            q = quaternion_from_euler(0, 0, yaw)

            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"

            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

            msg.twist.twist.linear.x = v
            msg.twist.twist.angular.z = w

            self.odom_pub.publish(msg)

        except Exception:
            pass

def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
