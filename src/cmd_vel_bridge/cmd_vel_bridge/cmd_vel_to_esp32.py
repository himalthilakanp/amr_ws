#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class CmdVelToESP32(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_esp32')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # ---------------- SERIAL ----------------
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            time.sleep(2.0)
            self.get_logger().info(f'Connected to ESP32 on {port}')
        except Exception as e:
            self.get_logger().fatal(f'Serial open failed: {e}')
            raise SystemExit

        # ---------------- QoS (CRITICAL FIX) ----------------
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ---------------- SUBSCRIBER ----------------
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos
        )

        # ---------------- STATE ----------------
        self.last_cmd = 'S'
        self.deadzone = 0.05

        self.get_logger().info('cmd_vel → ESP32 bridge READY')

    # ---------------- CALLBACK ----------------
    def cmd_vel_callback(self, msg):
        lin = msg.linear.x
        ang = msg.angular.z

        # Decide command
        if abs(lin) < self.deadzone and abs(ang) < self.deadzone:
            cmd = 'S'
        elif abs(lin) >= abs(ang):
            cmd = 'F' if lin > 0.0 else 'B'
        else:
            cmd = 'L' if ang > 0.0 else 'R'

        # Send only if changed
        if cmd != self.last_cmd:
            try:
                self.ser.write(cmd.encode())
                self.last_cmd = cmd
                self.get_logger().info(
                    f'Sent: {cmd} | linear.x={lin:.2f}, angular.z={ang:.2f}'
                )
            except Exception as e:
                self.get_logger().error(f'Serial write failed: {e}')


def main():
    rclpy.init()
    node = CmdVelToESP32()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
