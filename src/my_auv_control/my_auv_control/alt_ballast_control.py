#!/usr/bin/env python3
"""Alternative Altitude & Pitch Trim via Ballast Tanks (v50.1)
Standalone node — controls depth and pitch using wrench commands.

Controls:
  W/S — target depth ±0.5 m
  A/D — target pitch ±2°
  Q   — reset to neutral
  T   — toggle PID ON/OFF
  SPACE — hold current depth
  ESC — quit
"""
import sys, math, termios, tty, select, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

P_Z0  = 101325.0
RHO_G = 9810.0


class AltBallastControl(Node):

    def __init__(self):
        super().__init__('alt_ballast_control')

        self.declare_parameter('kp_z', 120.0)
        self.declare_parameter('ki_z', 3.0)
        self.declare_parameter('kd_z', 250.0)
        self.declare_parameter('kp_pitch', 50.0)
        self.declare_parameter('kd_pitch', 25.0)
        self.declare_parameter('max_force', 500.0)
        self.declare_parameter('max_torque', 80.0)

        self.kp_z  = self.get_parameter('kp_z').value
        self.ki_z  = self.get_parameter('ki_z').value
        self.kd_z  = self.get_parameter('kd_z').value
        self.kp_p  = self.get_parameter('kp_pitch').value
        self.kd_p  = self.get_parameter('kd_pitch').value
        self.max_f = self.get_parameter('max_force').value
        self.max_t = self.get_parameter('max_torque').value

        self.depth = 0.0
        self.vel_z = 0.0
        self.rpy = [0.0, 0.0, 0.0]
        self.rpy_rate = [0.0, 0.0, 0.0]
        self.prev_rpy = [0.0, 0.0, 0.0]
        self.target_depth = 0.0
        self.target_pitch = 0.0
        self.pid_on = False
        self.data_ok = False
        self._iz = 0.0

        self.create_subscription(Odometry,
            '/model/submarine/odometry', self._odom_cb, 10)
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32,
            '/model/submarine/pressure', self._press_cb, qos)

        self.pub_wrench = self.create_publisher(WrenchStamped,
            '/model/submarine/link/body/wrench', 10)

        self.fd = sys.stdin.fileno()
        self.old_termios = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self._loop)

        sys.stdout.write(
            "\n\x1b[1m=== Alt Ballast Control v50.1 ===\x1b[0m\n"
            " W/S=depth±0.5m  A/D=pitch±2°  Q=neutral\n"
            " T=PID  SPACE=hold  ESC=quit\n")
        sys.stdout.flush()

    def _press_cb(self, msg):
        self.depth = (P_Z0 - max(0.0, msg.data)) / RHO_G

    def _odom_cb(self, msg):
        self.data_ok = True
        self.vel_z = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        self.rpy[0] = math.atan2(2*(q.w*q.x + q.y*q.z),
                                 1 - 2*(q.x**2 + q.y**2))
        self.rpy[1] = math.asin(max(-1.0, min(1.0,
                                 2*(q.w*q.y - q.z*q.x))))
        self.rpy[2] = math.atan2(2*(q.w*q.z + q.x*q.y),
                                 1 - 2*(q.y**2 + q.z**2))

    def _loop(self):
        dt = self.dt
        self.rpy_rate = [
            (self.rpy[i] - self.prev_rpy[i]) / dt for i in range(3)]
        self.prev_rpy = list(self.rpy)

        if select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if   k == 'w': self.target_depth -= 0.5
            elif k == 's': self.target_depth += 0.5
            elif k == 'a': self.target_pitch -= math.radians(2)
            elif k == 'd': self.target_pitch += math.radians(2)
            elif k == 'q': self.target_depth = 0.0; self.target_pitch = 0.0
            elif k == 't': self.pid_on = not self.pid_on
            elif k == ' ':
                self.target_depth = self.depth
                self.target_pitch = 0.0
            elif k in ('\x1b',):
                self._cleanup()
                raise SystemExit

        fz = 0.0
        ty = 0.0

        if self.pid_on and self.data_ok:
            z_err = self.depth - self.target_depth
            self._iz += z_err * dt
            self._iz = max(-300.0, min(300.0, self._iz))
            if abs(z_err) < 0.15:
                self._iz *= 0.95

            fz = (self.kp_z * (-z_err)
                  + self.ki_z * (-self._iz)
                  + self.kd_z * (-self.vel_z))
            fz = max(-self.max_f, min(self.max_f, fz))

            pitch_err = self.target_pitch - self.rpy[1]
            ty = self.kp_p * pitch_err + self.kd_p * (-self.rpy_rate[1])
            ty = max(-self.max_t, min(self.max_t, ty))

        w = WrenchStamped()
        w.wrench.force.z = fz
        w.wrench.torque.y = ty
        self.pub_wrench.publish(w)

        mode = "PID" if self.pid_on else "MAN"
        dp = math.degrees(self.rpy[1])
        dtgt = math.degrees(self.target_pitch)
        sys.stdout.write(
            f"\r\033[K[{mode}] "
            f"Z:{self.depth:+.2f}m→{self.target_depth:+.1f}m "
            f"P:{dp:+.1f}°→{dtgt:+.1f}° "
            f"F:{fz:+.0f}N T:{ty:+.0f}Nm "
            f"Vz:{self.vel_z:+.2f}")
        sys.stdout.flush()

    def _cleanup(self):
        self.pub_wrench.publish(WrenchStamped())
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_termios)
        sys.stdout.write("\n🛑 Shutdown.\n")


def main():
    rclpy.init()
    node = AltBallastControl()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
