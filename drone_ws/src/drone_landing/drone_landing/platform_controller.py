"""
platform_controller.py
Platform moves in ALL 3 AXES — X (sine), Y (cosine), Z (slow bob).
This makes landing much more challenging and realistic.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class PlatformController(Node):
    def __init__(self):
        super().__init__('platform_controller')

        self.declare_parameter('amplitude',  1.5)   # XY amplitude metres
        self.declare_parameter('z_amplitude',0.3)   # Z bob amplitude metres
        self.declare_parameter('frequency',  0.15)  # Hz
        self.declare_parameter('pattern',   'sine')

        self.amp   = self.get_parameter('amplitude').value
        self.z_amp = self.get_parameter('z_amplitude').value
        self.freq  = self.get_parameter('frequency').value
        self.pat   = self.get_parameter('pattern').value

        self.pub = self.create_publisher(Twist, '/platform/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.move)
        self.t = 0.0
        self.get_logger().info(f'Platform controller: pattern={self.pat} '
                               f'amp={self.amp} z_amp={self.z_amp} freq={self.freq}Hz')

    def move(self):
        msg  = Twist()
        w    = 2 * math.pi * self.freq
        t    = self.t

        # ── XY motion ─────────────────────────────────────────────── #
        if self.pat == 'sine':
            # X: sine,  Y: cosine  → circular-ish in XY plane
            msg.linear.x = self.amp * w * math.cos(w * t)
            msg.linear.y = self.amp * w * 0.6 * math.cos(w * t + math.pi/3)

        elif self.pat == 'circle':
            msg.linear.x = self.amp * w * (-math.sin(w * t))
            msg.linear.y = self.amp * w * math.cos(w * t)

        elif self.pat == 'lissajous':
            msg.linear.x = self.amp * w * math.cos(w * t)
            msg.linear.y = self.amp * w * math.cos(2 * w * t)

        # ── Z bobbing (slow, small) ───────────────────────────────── #
        # Platform bobs up and down at half frequency
        msg.linear.z = self.z_amp * w * 0.5 * math.cos(0.5 * w * t)

        msg.angular.z = 0.0
        self.pub.publish(msg)
        self.t += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = PlatformController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
