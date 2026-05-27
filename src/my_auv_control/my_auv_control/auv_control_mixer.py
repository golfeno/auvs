#!/usr/bin/env python3
"""AUV Control Mixer | Odometry-only Angles & Rates | 1 line output"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
import sys, termios, tty, select, math, time

class AUVUnifiedNode(Node):
    def __init__(self):
        super().__init__('auv_unified')
        self.declare_parameter('kp_roll', 2.5); self.declare_parameter('kd_roll', 1.0)
        self.declare_parameter('kp_pitch', 2.5); self.declare_parameter('kd_pitch', 1.0)
        self.declare_parameter('max_rudder', 0.6); self.declare_parameter('max_thrust', 25.0)
        self.declare_parameter('max_heave_force', 300.0); self.declare_parameter('ballast_torque', 25.0)
        
        self.kp_r = self.get_parameter('kp_roll').value; self.kd_r = self.get_parameter('kd_roll').value
        self.kp_p = self.get_parameter('kp_pitch').value; self.kd_p = self.get_parameter('kd_pitch').value
        self.max_rudder = self.get_parameter('max_rudder').value; self.max_thrust = self.get_parameter('max_thrust').value
        self.max_heave = self.get_parameter('max_heave_force').value; self.b_torque = self.get_parameter('ballast_torque').value

        self.pub_lt = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_rt = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_hl = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_hr = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)
        self.pub_wrench = self.create_publisher(WrenchStamped, '/model/submarine/link/body/wrench', 10)

        self.create_subscription(Odometry, '/model/submarine/odometry', self.odom_cb, 10)

        # Commands
        self.cmd_thrust=0.0; self.cmd_diff=0.0; self.cmd_yaw=0.0; self.cmd_pitch=0.0
        self.cmd_ballast_pitch=0.0; self.cmd_ballast_heave=0.0
        # Telemetry
        self.tel_roll=0.0; self.tel_pitch=0.0; self.tel_yaw=0.0
        self.prev_roll=0.0; self.prev_pitch=0.0; self.prev_yaw=0.0
        self.tel_roll_rate=0.0; self.tel_pitch_rate=0.0; self.tel_yaw_rate=0.0
        self.tel_vx=0.0; self.tel_vy=0.0; self.tel_vz=0.0
        
        self.stab_on = False; self.shutdown = False
        self.data_ok = False; self.start_time = time.time()

        self.fd = sys.stdin.fileno(); self.old = termios.tcgetattr(self.fd); tty.setraw(self.fd)
        self.timer = self.create_timer(0.05, self.loop) # 20 Гц

    def odom_cb(self, msg):
        self.data_ok = True
        self.tel_vx = msg.twist.twist.linear.x
        self.tel_vy = msg.twist.twist.linear.y
        self.tel_vz = msg.twist.twist.linear.z
        
        q = msg.pose.pose.orientation
        r = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        p = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        y = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        
        dt = 0.05
        self.tel_roll_rate  = (r - self.prev_roll) / dt
        self.tel_pitch_rate = (p - self.prev_pitch) / dt
        self.tel_yaw_rate   = (y - self.prev_yaw) / dt
        self.prev_roll, self.prev_pitch, self.prev_yaw = r, p, y
        self.tel_roll, self.tel_pitch, self.tel_yaw = r, p, y

    def loop(self):
        if self.shutdown: return
        if not self.data_ok and (time.time() - self.start_time) > 4.0:
            sys.stdout.write("\r⚠️  NO ODOMETRY DATA! Check Gazebo/Bridge. Waiting...")
            sys.stdout.flush()
            if select.select([sys.stdin], [], [], 0)[0]: sys.stdin.read(1)
            return

        if select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if k=='i': self.cmd_thrust = max(-self.max_thrust, min(self.max_thrust, self.cmd_thrust+2.0))
            elif k=='k': self.cmd_thrust = max(-self.max_thrust, min(self.max_thrust, self.cmd_thrust-2.0))
            elif k=='j': self.cmd_diff = max(-self.max_thrust, min(self.max_thrust, self.cmd_diff+2.0))
            elif k=='l': self.cmd_diff = max(-self.max_thrust, min(self.max_thrust, self.cmd_diff-2.0))
            elif k=='a': self.cmd_yaw = max(-self.max_rudder, min(self.max_rudder, self.cmd_yaw+0.1))
            elif k=='d': self.cmd_yaw = max(-self.max_rudder, min(self.max_rudder, self.cmd_yaw-0.1))
            elif k=='w': self.cmd_pitch = max(-self.max_rudder, min(self.max_rudder, self.cmd_pitch+0.1))
            elif k=='s': self.cmd_pitch = max(-self.max_rudder, min(self.max_rudder, self.cmd_pitch-0.1))
            elif k=='u': self.cmd_ballast_pitch = max(-1.0, min(1.0, self.cmd_ballast_pitch-0.1))
            elif k=='o': self.cmd_ballast_pitch = max(-1.0, min(1.0, self.cmd_ballast_pitch+0.1))
            elif k=='f': self.cmd_ballast_heave = max(-self.max_heave, min(self.max_heave, self.cmd_ballast_heave-50.0))
            elif k=='b': self.cmd_ballast_heave = max(-self.max_heave, min(self.max_heave, self.cmd_ballast_heave+50.0))
            elif k=='t': self.stab_on = not self.stab_on
            elif k==' ': self._emergency_stop()
            elif k in ['\x1b','q']: self.shutdown = True

        stab_r = stab_p = 0.0
        if self.stab_on:
            stab_r = max(-0.25, min(0.25, (self.kp_r*(-self.tel_roll)-self.kd_r*self.tel_roll_rate)*0.4))
            stab_p = max(-0.25, min(0.25, (self.kp_p*(-self.tel_pitch)-self.kd_p*self.tel_pitch_rate)*0.4))

        rud_h = max(-self.max_rudder, min(self.max_rudder, self.cmd_pitch + self.cmd_ballast_pitch + stab_p))
        rud_v = max(-self.max_rudder, min(self.max_rudder, self.cmd_yaw + stab_r*0.3))
        thr_l = -max(-self.max_thrust, min(self.max_thrust, self.cmd_thrust + self.cmd_diff))
        thr_r = -max(-self.max_thrust, min(self.max_thrust, self.cmd_thrust - self.cmd_diff))

        self.pub_hl.publish(Float64(data=rud_h)); self.pub_hr.publish(Float64(data=rud_h))
        self.pub_vert.publish(Float64(data=rud_v))
        self.pub_lt.publish(Float64(data=thr_l)); self.pub_rt.publish(Float64(data=thr_r))
        w = WrenchStamped()
        w.wrench.force.z = self.cmd_ballast_heave
        w.wrench.torque.y = self.cmd_ballast_pitch * self.b_torque
        self.pub_wrench.publish(w)

        status = (f"\r\x1b[K[{'S' if self.stab_on else 'M'}] "
                  f"T:{self.cmd_thrust:+3.0f} D:{self.cmd_diff:+3.0f} | "
                  f"R:{rud_h:+.2f}/{rud_v:+.2f} | "
                  f"B:{self.cmd_ballast_heave:+4.0f}/{self.cmd_ballast_pitch:+.2f} | "
                  f"V:{self.tel_vx:+.2f}/{self.tel_vy:+.2f}/{self.tel_vz:+.2f} | "
                  f"W:{math.degrees(self.tel_roll_rate):+4.1f}/{math.degrees(self.tel_pitch_rate):+4.1f}/{math.degrees(self.tel_yaw_rate):+4.1f} | "
                  f"A:{math.degrees(self.tel_roll):+4.1f}/{math.degrees(self.tel_pitch):+4.1f}/{math.degrees(self.tel_yaw):+4.1f}")
        sys.stdout.write(status)
        sys.stdout.flush()

    def _emergency_stop(self):
        self.cmd_thrust=0.0; self.cmd_diff=0.0; self.cmd_pitch=0.0; self.cmd_yaw=0.0
        self.cmd_ballast_pitch=0.0; self.cmd_ballast_heave=0.0
        zero = Float64(data=0.0)
        for p in [self.pub_lt, self.pub_rt, self.pub_vert, self.pub_hl, self.pub_hr]: p.publish(zero)
        self.pub_wrench.publish(WrenchStamped())

    def cleanup(self):
        self._emergency_stop(); termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        sys.stdout.write("\n🛑 Shutdown complete.\n")

    def run(self):
        try:
            while rclpy.ok() and not self.shutdown: rclpy.spin_once(self, timeout_sec=0.01)
        finally: self.cleanup()

def main(args=None):
    rclpy.init(args=args); node = AUVUnifiedNode()
    node.get_logger().info("🎮 I/K=Thrust, J/L=Diff, W/S=Pitch, A/D=Yaw, U/O=Trim, F/B=Heave, T=STAB, SPACE=STOP, ESC=Quit")
    try: node.run()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
