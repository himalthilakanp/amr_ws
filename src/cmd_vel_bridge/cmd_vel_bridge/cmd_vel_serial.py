import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

class CmdVelSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial')
        self.create_subscription(
            Twist,
            '/cmd_vel_smoothed',
            self.cb,
            10
        )

    def cb(self, msg):
        data = f"{msg.linear.x:.3f} {msg.angular.z:.3f}\n"
        ser.write(data.encode())
    def destroy_node(self):
        try:
            ser.write(b"0.0 0.0\n")   # HARD STOP
        except:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = CmdVelSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
