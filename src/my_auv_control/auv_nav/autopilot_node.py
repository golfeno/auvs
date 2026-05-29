#!/usr/bin/env python3
"""AUV Autopilot v51.2 — clean, no ballast zeroing."""
import sys, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from .models import MotorMode, DepthMode, ActuatorCommands, Phys
from .sensor_fusion import SensorFusion
from .phase_manager import PhaseManager
from .heading import HeadingController
from .depth_rudder import DepthRudderController
from .depth_ballast import DepthBallastController
from .roll import RollController
from .telemetry import Telemetry


class AUVAutopilotNode(Node):
    def __init__(self, wps, mm, dm):
        super().__init__('auv_ctrl')
        self.mm = mm; self.dm = dm; self.tw = len(wps)
        self.pm = PhaseManager(wps, mm, dm)
        self.sf = SensorFusion(self)
        self.heading = HeadingController()
        self.depth_r = DepthRudderController()
        self.depth_b = DepthBallastController()
        self.roll = RollController()
        self.tl = Telemetry(self)

        self.pub_lt   = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_rt   = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_vert_top = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder_top/cmd_position', 10)
        self.pub_hl   = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_hr   = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)
        self.pub_hfl  = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_front_left/cmd_position', 10)
        self.pub_hfr  = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_front_right/cmd_position', 10)

        self.pub_b = []
        for i in range(1, 5):
            self.pub_b.append(self.create_publisher(
                Float64, f'/model/submarine/ballast_{i}/volume', 10))

        self._b = 0.0
        self.timer = self.create_timer(Phys.DT, self._loop)
        t0 = self.get_clock().now().nanoseconds / 1e9
        self.pm.init_wp(t0, [0.0, 0.0, 0.0])

    def _loop(self):
        # v51.4
        if self.pm.state == 'FINISH':
            return

        t = self.get_clock().now().nanoseconds / 1e9

        # Sensors
        self.sf.update(self.pm.target, Phys.DT)
        self.pm.update_drift(self.sf.state, t, Phys.DT)

        s = self.sf.state
        ph = self.pm.evaluate(s, t)
        tp = self.pm.params(s)

        # Reset on phase change
        if self.pm.need_pid_reset:
            self.heading.reset()
            self.depth_r.reset()
            self.depth_b.reset()
            self.roll.reset()
            self._b = 0.0
            self.pm.need_pid_reset = False

        cmd = ActuatorCommands()

        # Heading — always (нижний вертикальный руль)
        cmd.rv = self.heading.compute(s, ph, Phys.DT, False)
        # Roll — верхний вертикальный руль (отдельный канал, НЕ инвертируется)
        cmd.rvt = self.roll.vertical_top(s, ph, Phys.DT)

        # Depth + roll через все 4 горизонтальных руля
        if self.dm in (DepthMode.RUDDER, DepthMode.BOTH):
            cmd.hl, cmd.hr, cmd.hfl, cmd.hfr = self.depth_r.compute(s, ph, Phys.DT)
        if self.dm in (DepthMode.BALLAST, DepthMode.BOTH):
            cmd.ballast_volume = self.depth_b.compute(s, ph, Phys.DT)

        # Thrust with slew limiting
        slew_map = {'NAV': 6.0, 'Z_CORRIDOR': 6.0, 'Z_STAB': 5.0, 'APPROACH': 8.0, 'HOVER_STAB': 0.0, 'FINISH': 0.0}
        sl = slew_map.get(ph, 6.0)
        d = sl * Phys.DT
        tgt = tp.get('bs', 0.0)
        self._b = self._b + max(-d, min(d, tgt - self._b))

        if ph in ('HOVER_STAB', 'FINISH'):
            cmd.lt = 0.0; cmd.rt = 0.0
        elif self.mm == MotorMode.SINGLE:
            cmd.lt = self._b; cmd.rt = self._b
        else:
            yd = tp.get('yd', 0.0)
            cmd.lt = self._b + yd
            cmd.rt = self._b - yd

        # Publish
        self._pub(cmd)
        self.tl.analytics(s, ph, self.pm.wp_idx)

        # Arrival
        if ph == 'HOVER_STAB' and abs(s.vel) < 0.25 and abs(s.z_err) < 0.5:
            self.tl.log_wp(self.pm.wp_idx + 1)
            self.pm.wp_idx += 1
            if self.pm.wp_idx < self.tw:
                self.heading.reset(); self.depth_r.reset(); self.depth_b.reset(); self.roll.reset()
                self._b = 0.0
                self.pm.init_wp(t, s.pos)
            else:
                self.pm.state = 'FINISH'
                self._pub(ActuatorCommands())
                sys.stdout.write("\n[v51.2] ✓ Миссия завершена! " + str(self.tw) + " точек.\n")
                sys.stdout.flush()
                raise SystemExit
        else:
            self.tl.log(s, ph, self.pm.wp_idx, t, self.tw, cmd)

    def _pub(self, cmd):
        self.pub_lt.publish(Float64(data=cmd.lt))
        self.pub_rt.publish(Float64(data=cmd.rt))
        self.pub_vert.publish(Float64(data=cmd.rv))
        # Верхний вертикальный руль — стабилизация крена (не инвертируется)
        self.pub_vert_top.publish(Float64(data=cmd.rvt))
        # Кормовые горизонтальные рули
        self.pub_hl.publish(Float64(data=cmd.hl))
        self.pub_hr.publish(Float64(data=cmd.hr))
        # Носовые горизонтальные рули — рассчитаны контроллером
        # (канал глубины зеркальный, канал крена сонаправленный)
        self.pub_hfl.publish(Float64(data=cmd.hfl))
        self.pub_hfr.publish(Float64(data=cmd.hfr))
        if self.dm in (DepthMode.BALLAST, DepthMode.BOTH):
            for pub in self.pub_b:
                pub.publish(Float64(data=cmd.ballast_volume * Phys.MAX_BALLAST_VOL))


def load_waypoints(filepath):
    wps = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) == 3:
                wps.append(tuple(map(float, parts)))
    return wps


def _ask(p, v, d=None):
    while True:
        r = input(p).strip()
        if r == '' and d is not None:
            return d
        try:
            n = int(r)
            if n in v:
                return n
        except ValueError:
            pass


def main():
    print("=" * 50)
    print("  AUV Autopilot v51.4 — BALLAST: stop if Z off, cruise if Z ok")
    print("=" * 50)

    default_file = os.path.expanduser("~/auv/waypoints.txt")
    wps = []
    if os.path.exists(default_file):
        print(f"\n  Файл: {default_file}")
        use = input("  Использовать? [Y/n]: ").strip().lower()
        if use in ('', 'y', 'yes'):
            wps = load_waypoints(default_file)
            print(f"  Загружено {len(wps)} точек:")
            for i, w in enumerate(wps, 1):
                print(f"    {i}: ({w[0]}, {w[1]}, {w[2]})")

    if not wps:
        print("\nWaypoints (X Y Z). Пустая = старт.")
        i = 1
        while True:
            try:
                inp = input(f"  Точка {i}: ").strip()
                if not inp:
                    if wps: break
                    continue
                p = inp.split()
                if len(p) == 3:
                    wps.append(tuple(map(float, p))); i += 1
            except ValueError:
                pass

    mm = _ask("\nДвигатели [1=синхрон / 2=дифференциал] (2): ", {1, 2}, 2)
    motor = MotorMode.SINGLE if mm == 1 else MotorMode.DUAL
    if motor == MotorMode.SINGLE:
        print("  ⚠  Оба движка синхронно, курс через вертик. руль.")

    dm = _ask("Глубина [1=рули / 2=балласты / 3=оба] (1): ", {1, 2, 3}, 1)
    depth = {1: DepthMode.RUDDER, 2: DepthMode.BALLAST, 3: DepthMode.BOTH}[dm]
    print(f"\n  → Двигатели: {motor.name}  |  Глубина: {depth.name}")

    rclpy.init()
    node = AUVAutopilotNode(wps, motor, depth)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node._pub(ActuatorCommands())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
