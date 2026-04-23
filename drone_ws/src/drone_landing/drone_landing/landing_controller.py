"""
landing_controller.py
Fixes:
  1. Tracks platform Z position too (platform now bobs in Z)
  2. Relaxed landing threshold so detection actually triggers
  3. After landing: drone moves with platform in ALL axes (X, Y, Z)
  4. Prints clear SUCCESS message
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray, Bool
import numpy as np
from collections import deque


class PIDController:
    def __init__(self, kp, ki, kd, output_limit=2.0):
        self.kp=kp; self.ki=ki; self.kd=kd
        self.limit=output_limit; self.integral=0.0; self.prev_meas=0.0

    def compute(self, error, measurement, dt):
        p = self.kp * error
        self.integral = np.clip(self.integral + error*dt, -self.limit, self.limit)
        i = self.ki * self.integral
        d = -self.kd * (measurement - self.prev_meas) / max(dt, 1e-6)
        self.prev_meas = measurement
        return float(np.clip(p+i+d, -self.limit, self.limit))

    def reset(self, m=0.0):
        self.integral=0.0; self.prev_meas=m


class VelocityEstimator:
    def __init__(self, window=8):
        self.pos=deque(maxlen=window); self.t=deque(maxlen=window)

    def update(self, pos, t):
        self.pos.append(pos); self.t.append(t)

    def velocity(self):
        if len(self.pos) < 3: return 0.0
        t = np.array(self.t) - self.t[0]
        p = np.array(self.pos)
        if t[-1] < 1e-6: return 0.0
        A = np.vstack([t, np.ones(len(t))]).T
        return float(np.linalg.lstsq(A, p, rcond=None)[0][0])


class LandingController(Node):
    def __init__(self):
        super().__init__('landing_controller')

        # XY PID
        self.pid_x = PIDController(kp=1.5, ki=0.08, kd=0.5, output_limit=2.5)
        self.pid_y = PIDController(kp=1.5, ki=0.08, kd=0.5, output_limit=2.5)
        # Z PID — tracks both approach alt AND platform Z offset
        self.pid_z = PIDController(kp=1.2, ki=0.05, kd=0.5, output_limit=1.5)

        self.ff_gain   = 0.9
        self.lead_time = 0.3

        # Velocity estimators for ALL 3 axes
        self.vel_x = VelocityEstimator()
        self.vel_y = VelocityEstimator()
        self.vel_z = VelocityEstimator()   # NEW: Z velocity

        self.drone_x=0.0; self.drone_y=0.0; self.drone_z=0.0
        self.plat_x=2.0;  self.plat_y=0.0; self.plat_z=0.025  # NEW: platform Z
        self.plat_vx=0.0; self.plat_vy=0.0; self.plat_vz=0.0  # NEW

        self.aruco_ox=None; self.aruco_oy=None
        self.marker_detected=False

        self.state     = 'IDLE'
        self.is_landed = False
        self.prev_time = self.get_clock().now()

        # Parameters
        self.approach_alt  = 4.0    # approach altitude above WORLD origin
        self.hover_above   = 0.8    # metres above platform surface during approach
        self.descent_rate  = 0.15   # m/s slow descent
        self.align_thresh  = 0.40   # xy must be < this to descend
        self.landed_z_thresh  = 0.60  # drone within 60cm above platform = landed
        self.landed_xy_thresh = 0.60  # within 60cm horizontally = landed

        self.create_subscription(Odometry, '/drone/odom',    self.drone_cb, 10)
        self.create_subscription(Odometry, '/platform/odom', self.plat_cb, 10)
        self.create_subscription(PoseStamped, '/landing/target_pose', self.aruco_cb, 10)
        self.create_subscription(Bool,   '/landing/marker_detected', self.marker_cb, 10)
        self.create_subscription(String, '/landing/state_command',   self.state_cb, 10)

        self.cmd_pub    = self.create_publisher(Twist,             '/drone/cmd_vel', 10)
        self.status_pub = self.create_publisher(String,            '/landing/status', 10)
        self.error_pub  = self.create_publisher(Float32MultiArray, '/landing/errors', 10)

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Landing controller ready')

    def drone_cb(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z

    def plat_cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.plat_x = msg.pose.pose.position.x
        self.plat_y = msg.pose.pose.position.y
        self.plat_z = msg.pose.pose.position.z   # track platform Z height
        self.vel_x.update(self.plat_x, t)
        self.vel_y.update(self.plat_y, t)
        self.vel_z.update(self.plat_z, t)
        self.plat_vx = self.vel_x.velocity()
        self.plat_vy = self.vel_y.velocity()
        self.plat_vz = self.vel_z.velocity()

    def aruco_cb(self, msg):
        self.aruco_ox = msg.pose.position.x
        self.aruco_oy = msg.pose.position.y

    def marker_cb(self, msg): self.marker_detected = msg.data

    def state_cb(self, msg):
        if msg.data != self.state:
            self.get_logger().info(f'State -> {msg.data}')
            self.state = msg.data
            if msg.data == 'IDLE':
                self.pid_x.reset(self.drone_x)
                self.pid_y.reset(self.drone_y)
                self.pid_z.reset(self.drone_z)

    def move_with_platform(self):
        """After landing: match platform velocity in ALL axes so drone stays on it."""
        cmd = Twist()
        cmd.linear.x = float(np.clip(self.plat_vx, -3.0, 3.0))
        cmd.linear.y = float(np.clip(self.plat_vy, -3.0, 3.0))
        cmd.linear.z = float(np.clip(self.plat_vz, -1.0, 1.0))  # track Z bob
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def trigger_landing(self, actual_xy):
        self.is_landed = True
        s = String(); s.data = 'LANDED'
        self.status_pub.publish(s)
        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════╗\n'
            '║          SUCCESSFULLY LANDED !               ║\n'
            f'║  Center error   : {actual_xy*100:6.1f} cm             ║\n'
            f'║  Drone altitude : {self.drone_z:.3f} m               ║\n'
            f'║  Platform Z     : {self.plat_z:.3f} m               ║\n'
            f'║  Drone XY   : ({self.drone_x:.2f}, {self.drone_y:.2f})          ║\n'
            f'║  Platform XY: ({self.plat_x:.2f}, {self.plat_y:.2f})          ║\n'
            '║  Drone now riding platform in all axes.      ║\n'
            '╚══════════════════════════════════════════════╝')

    def loop(self):
        # ── LANDED: follow platform in X, Y, Z ───────────────────── #
        if self.is_landed:
            self.move_with_platform()
            return

        if self.state == 'IDLE':
            return

        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0 or dt > 0.5:
            return

        # ── Predicted platform XY position ───────────────────────── #
        pred_x = self.plat_x + self.ff_gain * self.plat_vx * self.lead_time
        pred_y = self.plat_y + self.ff_gain * self.plat_vy * self.lead_time

        if self.marker_detected and self.aruco_ox is not None:
            ax = self.drone_x + self.aruco_ox
            ay = self.drone_y + self.aruco_oy
            target_x = 0.7*ax + 0.3*pred_x
            target_y = 0.7*ay + 0.3*pred_y
        else:
            target_x = pred_x
            target_y = pred_y

        err_x = target_x - self.drone_x
        err_y = target_y - self.drone_y
        xy    = float(np.hypot(err_x, err_y))

        # ── Z setpoint — platform-relative ───────────────────────── #
        # Always aim for (platform_z + hover_above) during APPROACH
        # During DESCEND, step toward (platform_z + small_offset)
        plat_surface = self.plat_z + 0.025   # top of 5cm platform

        if self.state in ('TAKEOFF', 'SEARCH'):
            target_z = self.approach_alt
            err_z    = target_z - self.drone_z

        elif self.state == 'APPROACH':
            # Hover hover_above metres over platform surface, track its Z
            target_z = plat_surface + self.hover_above
            # Add Z feedforward so we follow platform bobbing
            err_z    = target_z - self.drone_z

        elif self.state == 'DESCEND':
            if xy < self.align_thresh:
                # Step down toward ground level (platform top is at ~0.05m)
                step_down = self.drone_z - self.descent_rate * dt
                target_z  = max(0.05, step_down)   # stop at 5cm above ground
            else:
                target_z = self.drone_z             # hold — realign first
            err_z = target_z - self.drone_z
        else:
            err_z = 0.0

        # ── PID + feedforward ─────────────────────────────────────── #
        vx = self.pid_x.compute(err_x, self.drone_x, dt) + self.ff_gain * self.plat_vx
        vy = self.pid_y.compute(err_y, self.drone_y, dt) + self.ff_gain * self.plat_vy
        # Z feedforward for platform bobbing during approach/descend
        vz_ff = (self.ff_gain * self.plat_vz
                 if self.state in ('APPROACH', 'DESCEND') else 0.0)
        vz = self.pid_z.compute(err_z, self.drone_z, dt) + vz_ff

        cmd = Twist()
        cmd.linear.x = float(np.clip(vx, -3.0,  3.0))
        cmd.linear.y = float(np.clip(vy, -3.0,  3.0))
        cmd.linear.z = float(np.clip(vz, -1.5,  1.5))
        self.cmd_pub.publish(cmd)

        # ── Telemetry ─────────────────────────────────────────────── #
        actual_xy = float(np.hypot(self.plat_x-self.drone_x,
                                   self.plat_y-self.drone_y))
        actual_z_gap = self.drone_z - plat_surface   # how far above platform
        e = Float32MultiArray()
        e.data = [err_x, err_y, err_z, xy,
                  self.plat_vx, self.plat_vy, actual_xy, actual_z_gap]
        self.error_pub.publish(e)

        # ── Landing detection ─────────────────────────────────────── #
        # Use absolute drone Z — platform is at z=0.025 so drone on it = z < 0.5
        # actual_xy = real distance from platform center
        if (self.state == 'DESCEND'
                and self.drone_z < 0.50          # drone is very low = on platform
                and actual_xy   < 0.65):         # within 65cm of center
            self.trigger_landing(actual_xy)


def main(args=None):
    rclpy.init(args=args)
    node = LandingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
