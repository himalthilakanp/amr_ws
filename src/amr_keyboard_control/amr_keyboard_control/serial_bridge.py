import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.arduino = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)
        self.get_logger().info("Serial bridge connected to /dev/ttyACM1")

    def cmd_vel_callback(self, msg):
        """
        Convert Twist message to single-character commands:
        F: forward, B: backward, L: left, R: right, S: stop
        """
        cmd = 'S'  # Default stop

        if msg.linear.x > 0.0:
            cmd = 'F'
        elif msg.linear.x < 0.0:
            cmd = 'B'
        elif msg.angular.z > 0.0:
            cmd = 'L'
        elif msg.angular.z < 0.0:
            cmd = 'R'
        
        # Send single byte command to Arduino
        self.arduino.write(cmd.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
