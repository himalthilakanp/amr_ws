#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time

class SerialTest(Node):
    def __init__(self):
        super().__init__('serial_test')

        # Open ESP32 serial (same as echo test)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        time.sleep(2)  # let ESP32 boot

        self.get_logger().info("ESP32 connected, sending test command...")

        # Send FORWARD
        self.ser.write(b'F')
        self.get_logger().info("Sent: F (forward)")
        time.sleep(2)

        # Send STOP
        self.ser.write(b'S')
        self.get_logger().info("Sent: S (stop)")

        self.get_logger().info("Serial test complete")
        rclpy.shutdown()


def main():
    rclpy.init()
    SerialTest()

if __name__ == '__main__':
    main()
