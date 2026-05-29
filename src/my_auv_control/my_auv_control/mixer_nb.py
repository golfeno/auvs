#!/usr/bin/env python3
"""Mixer для submarine_nb (без балластов). Жёстко привязан к модели."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
import sys, termios, tty, select, math, time

M = 'submarine_nb'

class MixerNB(Node):
    def __init__(self):
        super().__init__('mixer_nb')
        self.pub_lt   = self.create_publisher(Float64, f'/model/{M}/joint/left_propeller_joint/cmd_force', 10)
        self.pub_rt   = self.create_publisher(Float64, f'/model/{M}/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, f'/model/{M}/joint/vertical_rudder/cmd_position', 10)
        self.pub_hl   = self.create_publisher(Float64, f'/model/{M}/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_hr   = self.create_publisher(Float64, f'/model/{M}/joint/horizontal_rudder_right/cmd_position', 10)
        self.pub_vert_top = self.create_publisher(Float64, f'/model/{M}/joint/vertical_rudder_top/cmd_position', 10)
        self.create_subscription(Odometry, f'/model/{M}/odometry', self._odom, 10)

        self.ct=0.0; self.cd=0.0; self.cy=0.0; self.cp=0.0
        self.vx=0.0; self.vy=0.0; self.vz=0.0
        self.roll=0.0; self.pitch=0.0; self.yaw=0.0
        self.prev_r=0.0; self.prev_p=0.0; self.prev_y=0.0
        self.rr=0.0; self.pr=0.0; self.yr=0.0
        self.ok = False; self.stab = False; self.dead = False
        self.start = time.time()
        self.mr = 0.6; self.mt = 25.0

        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        self.timer = self.create_timer(0.05, self._loop)

    def _odom(self, msg):
        self.ok = True
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        r = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        p = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        y = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        dt = 0.05
        self.rr = (r - self.prev_r) / dt
        self.pr = (p - self.prev_p) / dt
        self.yr = (y - self.prev_y) / dt
        self.prev_r, self.prev_p, self.prev_y = r, p, y
        self.roll, self.pitch, self.yaw = r, p, y

    def _loop(self):
        if self.dead:
            return

        # --- Клавиатура читается ВСЕГДА (даже без одометрии) ---
        if select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if   k == 'i': self.ct = max(-self.mt, min(self.mt, self.ct+2.0))
            elif k == 'k': self.ct = max(-self.mt, min(self.mt, self.ct-2.0))
            elif k == 'j': self.cd = max(-self.mt, min(self.mt, self.cd+2.0))
            elif k == 'l': self.cd = max(-self.mt, min(self.mt, self.cd-2.0))
            elif k == 'a': self.cy = max(-self.mr, min(self.mr, self.cy+0.1))
            elif k == 'd': self.cy = max(-self.mr, min(self.mr, self.cy-0.1))
            elif k == 'w': self.cp = max(-self.mr, min(self.mr, self.cp+0.1))
            elif k == 's': self.cp = max(-self.mr, min(self.mr, self.cp-0.1))
            elif k == 't': self.stab = not self.stab
            elif k == ' ':
                self.ct=self.cd=self.cy=self.cp=0.0
                for p in [self.pub_lt,self.pub_rt,self.pub_vert,self.pub_vert_top,self.pub_hl,self.pub_hr]:
                    p.publish(Float64(data=0.0))
            elif k in ('\x1b','q'):
                self.dead = True

        # Stabilization
        sp = sr = 0.0
        rvt = 0.0
        if self.stab and self.ok:
            sp = max(-0.25, min(0.25, (-2.5*self.pitch - 1.0*self.pr)*0.4))
            sr = max(-0.25, min(0.25, (-2.5*self.roll  - 1.0*self.rr)*0.4))
            # Верхний вертикальный руль — отдельный канал крена (НЕ инвертируется)
            rvt = max(-self.mr, min(self.mr, (-2.5*self.roll - 1.0*self.rr)))

        rh = max(-self.mr, min(self.mr, self.cp + sp))   # глубина (тангаж)
        roll_h = max(-self.mr, min(self.mr, sr))         # крен (дифференциально)
        rv = max(-self.mr, min(self.mr, self.cy))
        tl = -max(-self.mt, min(self.mt, self.ct + self.cd))
        tr = -max(-self.mt, min(self.mt, self.ct - self.cd))

        self.pub_hl.publish(Float64(data=rh - roll_h))
        self.pub_hr.publish(Float64(data=rh + roll_h))
        self.pub_vert.publish(Float64(data=rv))
        self.pub_vert_top.publish(Float64(data=rvt))
        self.pub_lt.publish(Float64(data=tl))
        self.pub_rt.publish(Float64(data=tr))

        sys.stdout.write(
            f"\r\x1b[K[{'S' if self.stab else 'M'}|{'ODO' if self.ok else 'no-odo'}] "
            f"T:{self.ct:+3.0f} D:{self.cd:+3.0f} | "
            f"R:{rh:+.2f}/{rv:+.2f} | "
            f"V:{self.vx:+.2f}/{self.vy:+.2f}/{self.vz:+.2f} | "
            f"A:{math.degrees(self.roll):+.0f}/{math.degrees(self.pitch):+.0f}/{math.degrees(self.yaw):+.0f}"
        )
        sys.stdout.flush()

    def cleanup(self):
        self.ct=self.cd=self.cy=self.cp=0.0
        for p in [self.pub_lt,self.pub_rt,self.pub_vert,self.pub_vert_top,self.pub_hl,self.pub_hr]:
            p.publish(Float64(data=0.0))
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        sys.stdout.write(f"\n🛑 {M} mixer stopped.\n")

    def run(self):
        try:
            while rclpy.ok() and not self.dead:
                rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            self.cleanup()

def main():
    rclpy.init()
    node = MixerNB()
    node.get_logger().info(f"🎮 {M} | I/K=Thrust J/L=Diff W/S=Pitch A/D=Yaw T=STAB SPACE=STOP")
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
