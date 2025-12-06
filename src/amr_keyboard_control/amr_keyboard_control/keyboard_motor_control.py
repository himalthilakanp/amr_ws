import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.1
        self.turn = 0.1
        self.get_logger().info("Use arrow keys to drive. Press CTRL+C to stop")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(3)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()

            if key == '\x1b[A':               # UP
                twist.linear.x = self.speed
                twist.angular.z = 0.0
            elif key == '\x1b[B':             # DOWN
                twist.linear.x = -self.speed
                twist.angular.z = 0.0
            elif key == '\x1b[C':             # RIGHT
                twist.angular.z = -self.turn
                twist.linear.x = 0.0
            elif key == '\x1b[D':             # LEFT
                twist.angular.z = self.turn
                twist.linear.x = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
