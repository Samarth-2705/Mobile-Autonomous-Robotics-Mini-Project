"""
state_machine.py
Fixes:
  1. LANDED state: platform KEEPS moving (drone rides on it)
  2. Prints clear "Successfully Landed" message
  3. Only drone Z is zeroed — XY tracks platform
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import threading, time, math


class State:
    IDLE='IDLE'; TAKEOFF='TAKEOFF'; SEARCH='SEARCH'
    APPROACH='APPROACH'; DESCEND='DESCEND'; LANDED='LANDED'; ABORT='ABORT'


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        self.current_state  = State.IDLE
        self.is_landed      = False
        self.marker_detected = False
        self.consecutive_det = 0

        self.drone_x=0.0; self.drone_y=0.0; self.drone_z=0.0
        self.platform_x=2.0; self.platform_y=0.0
        self.xy_error=999.0

        self.takeoff_alt        = 4.0
        self.approach_xy_thresh = 0.8
        self.search_timeout     = 180.0
        self.search_start_time  = None

        self.create_subscription(Bool,              '/landing/marker_detected', self.marker_cb, 10)
        self.create_subscription(Odometry,          '/drone/odom',              self.drone_cb, 10)
        self.create_subscription(Odometry,          '/platform/odom',           self.plat_cb, 10)
        self.create_subscription(String,            '/landing/status',          self.status_cb, 10)
        self.create_subscription(Float32MultiArray, '/landing/errors',          self.errors_cb, 10)

        self.state_pub = self.create_publisher(String, '/landing/state_command', 10)
        self.drone_cmd = self.create_publisher(Twist,  '/drone/cmd_vel', 10)
        # NOTE: no platform stop — platform keeps moving, drone rides on it

        self.timer = self.create_timer(0.1, self.run)
        self.get_logger().info('State machine ready')

    def marker_cb(self, msg):
        self.consecutive_det = (min(self.consecutive_det+1,20) if msg.data
                                else max(self.consecutive_det-1,0))
        self.marker_detected = self.consecutive_det >= 3

    def drone_cb(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z

    def plat_cb(self, msg):
        self.platform_x = msg.pose.pose.position.x
        self.platform_y = msg.pose.pose.position.y

    def status_cb(self, msg):
        if msg.data == 'LANDED' and not self.is_landed:
            self.is_landed = True
            self.transition(State.LANDED)

    def errors_cb(self, msg):
        if len(msg.data) >= 4:
            self.xy_error = msg.data[3]

    def transition(self, new_state):
        if new_state == self.current_state: return
        self.get_logger().info(f'{self.current_state} -> {new_state}')
        self.current_state = new_state
        m = String(); m.data = new_state
        self.state_pub.publish(m)

    def dist_to_platform(self):
        return math.hypot(self.platform_x - self.drone_x,
                          self.platform_y - self.drone_y)

    def run(self):
        # ── LANDED: platform still moves, drone rides on it ──────── #
        if self.is_landed:
            # landing_controller handles move_with_platform()
            # Just keep printing success every 10s so terminal is clear
            self.get_logger().info(
                'Successfully Landed — drone riding platform.',
                throttle_duration_sec=10.0)
            return

        if self.current_state == State.IDLE:
            return

        elif self.current_state == State.TAKEOFF:
            if self.drone_z >= self.takeoff_alt - 0.15:
                self.get_logger().info(f'Takeoff done z={self.drone_z:.2f}m')
                self.search_start_time = time.time()
                self.transition(State.SEARCH)
            else:
                cmd = Twist(); cmd.linear.z = 0.8
                self.drone_cmd.publish(cmd)

        elif self.current_state == State.SEARCH:
            elapsed = time.time() - self.search_start_time
            dist    = self.dist_to_platform()
            if elapsed > self.search_timeout:
                self.get_logger().warn('Search timeout — ABORT')
                self.transition(State.ABORT); return
            if dist < 3.0 and self.drone_z >= self.takeoff_alt - 0.5:
                self.get_logger().info(f'Platform in range dist={dist:.2f}m — APPROACH')
                self.transition(State.APPROACH); return
            ex = self.platform_x - self.drone_x
            ey = self.platform_y - self.drone_y
            speed = min(1.5, dist * 0.5)
            if dist > 0.1:
                cmd = Twist()
                cmd.linear.x = speed * ex / dist
                cmd.linear.y = speed * ey / dist
                cmd.linear.z = max(0.0, (self.takeoff_alt - self.drone_z) * 0.5)
                self.drone_cmd.publish(cmd)

        elif self.current_state == State.APPROACH:
            dist = self.dist_to_platform()
            if dist > 5.0:
                self.get_logger().warn(f'Lost platform dist={dist:.2f}m — SEARCH')
                self.search_start_time = time.time()
                self.transition(State.SEARCH); return
            if (self.xy_error < self.approach_xy_thresh and
                    self.drone_z >= self.takeoff_alt - 0.5):
                self.get_logger().info(f'Aligned xy_error={self.xy_error:.2f}m — DESCEND')
                self.transition(State.DESCEND)

        elif self.current_state == State.DESCEND:
            dist = self.dist_to_platform()
            if dist > 1.5 and self.drone_z > 1.0:
                self.get_logger().warn(f'Drifted dist={dist:.2f}m — APPROACH')
                self.transition(State.APPROACH)

        elif self.current_state == State.LANDED:
            # This block reached only first time before is_landed set
            pass

        elif self.current_state == State.ABORT:
            self.drone_cmd.publish(Twist())

    def start_mission(self):
        self.transition(State.TAKEOFF)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    threading.Thread(
        target=lambda: (time.sleep(2.0), node.start_mission()),
        daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
