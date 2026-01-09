#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanSanitizer(Node):
    def __init__(self):
        super().__init__('scan_sanitizer')
        self.sub = self.create_subscription(
            LaserScan,
            '/MS200/scan',
            self.cb,
            10
        )
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

    def cb(self, msg):
        clean = LaserScan()
        clean = msg
        clean.ranges = [
            msg.range_max if (math.isnan(r) or r < msg.range_min) else r
            for r in msg.ranges
        ]
        self.pub.publish(clean)

def main():
    rclpy.init()
    rclpy.spin(ScanSanitizer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
