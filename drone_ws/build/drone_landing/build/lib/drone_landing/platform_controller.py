import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class PlatformController(Node):
    def __init__(self):
        super().__init__('platform_controller')
        self.declare_parameter('amplitude', 1.5)   # metres
        self.declare_parameter('frequency', 0.2)   # Hz
        self.declare_parameter('pattern', 'sine')  # sine | circle | lissajous

        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        self.pattern   = self.get_parameter('pattern').value

        self.publisher_ = self.create_publisher(Twist, '/platform/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.move_platform)
        self.t = 0.0
        self.get_logger().info(f'Platform controller: pattern={self.pattern}')

    def move_platform(self):
        msg = Twist()
        omega = 2 * math.pi * self.frequency

        if self.pattern == 'sine':
            msg.linear.x = self.amplitude * omega * math.cos(omega * self.t)
            msg.linear.y = 0.0
        elif self.pattern == 'circle':
            msg.linear.x = self.amplitude * omega * (-math.sin(omega * self.t))
            msg.linear.y = self.amplitude * omega * math.cos(omega * self.t)
        elif self.pattern == 'lissajous':
            msg.linear.x = self.amplitude * omega * math.cos(omega * self.t)
            msg.linear.y = self.amplitude * omega * math.cos(2 * omega * self.t)

        msg.linear.z  = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.t += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = PlatformController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
