#!/usr/bin/env python3
"""AUV Autopilot v50.22 — drift via position, Z_HOLD, descent bias."""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
from .models import MotorMode, DepthMode, ActuatorCommands, Phys
from .sensor_fusion import SensorFusion
from .phase_manager import PhaseManager
from .pid_controller import PIDController
from .telemetry import Telemetry


class AUVAutopilotNode(Node):
    def __init__(self, wps, mm, dm):
        super().__init__('auv_ctrl')
        self.mm = mm; self.dm = dm; self.tw = len(wps)
        self.pm = PhaseManager(wps, mm)
        self.sf = SensorFusion(self)
        self.pc = PIDController(mm, dm)
        self.tl = Telemetry(self)

        self.pub_lt   = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_rt   = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_hl   = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_hr   = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)
        self.pub_wrench = self.create_publisher(WrenchStamped, '/model/submarine/link/body/wrench', 10)

        # Drift measurement
        self.drift_phase = True
        self._drift_pos0 = None
        self._drift_t0 = 0.0
        self.drift_vx = 0.0; self.drift_vz = 0.0
        self.DRIFT_DURATION = 5.0

        t0 = self.get_clock().now().nanoseconds / 1e9
        self._drift_start = t0
        self.timer = self.create_timer(Phys.DT, self._loop)
        sys.stdout.write("\r\033[K[v50.22] Замер дрейфа (5 сек)... ")
        sys.stdout.flush()

    def _loop(self):
        t = self.get_clock().now().nanoseconds / 1e9

        # ═══ DRIFT MEASUREMENT ═══
        if self.drift_phase:
            self.sf.update([0, 0, 0], Phys.DT)
            s = self.sf.state
            if self._drift_pos0 is None:
                self._drift_pos0 = list(s.pos)
                self._drift_t0 = t
            self._pub(ActuatorCommands())
            if t - self._drift_start >= self.DRIFT_DURATION:
                dt_m = t - self._drift_t0
                if dt_m > 0.1:
                    self.drift_vx = (s.pos[0] - self._drift_pos0[0]) / dt_m
                    self.drift_vz = (s.pos[2] - self._drift_pos0[2]) / dt_m
                self.pm.drift_vx = self.drift_vx
                self.pm.drift_vz = self.drift_vz
                self.drift_phase = False
                self.pm.init_wp(t, s.pos)
                sys.stdout.write("OK\n  Drift: Vx=" + format(self.drift_vx, '+.3f') + " Vz=" + format(self.drift_vz, '+.3f') + " м/с\n")
                sys.stdout.flush()
            return

        # ═══ NORMAL OPERATION ═══
        if self.pm.state == 'FINISH':
            return

        # Dynamic Z target only in Z_HOLD
        if self.pm.state == 'Z_HOLD':
            s_raw = self.sf.state
            dyn_z = self.pm.get_dynamic_z_target(s_raw)
            active_target = [self.pm.target[0], self.pm.target[1], dyn_z]
        else:
            active_target = self.pm.target

        self.sf.update(active_target, Phys.DT)
        self.pm.update_drift(self.sf.state, t, Phys.DT)

        s = self.sf.state
        ph = self.pm.evaluate(s, t)
        tp = self.pm.params(s)

        if self.pm.need_pid_reset:
            self.pc.reset()
            self.pm.need_pid_reset = False
            if ph == 'XY_FINAL' and self.pm.xy_sub == 'BRAKE':
                self.pc.force_zero_thrust()

        turning = (ph == 'XY_FINAL' and self.pm.xy_sub == 'TURN')
        cmd = self.pc.compute(s, ph, tp, Phys.DT, bo=False, turning=turning)

        self._pub(cmd)
        self.tl.analytics(s, ph, self.pm.wp_idx)

        if ph == 'HOVER_STAB' and abs(s.vel) < 0.25 and abs(s.z_err) < 0.5:
            sys.stdout.write("\n  ✓ Точка " + str(self.pm.wp_idx + 1))
            sys.stdout.flush()
            self.pm.wp_idx += 1
            if self.pm.wp_idx < self.tw:
                self.pc.reset(); self.pm.init_wp(t, s.pos)
            else:
                self.pm.state = 'FINISH'; self._pub(ActuatorCommands())
                sys.stdout.write("\n[v50.22] ✓ Миссия завершена! " + str(self.tw) + " точек.\n")
                sys.stdout.flush()
                raise SystemExit
        else:
            self.tl.log(s, ph, self.pm.wp_idx, t, self.tw, cmd)

    def _pub(self, c):
        self.pub_lt.publish(Float64(data=c.lt))
        self.pub_rt.publish(Float64(data=c.rt))
        self.pub_vert.publish(Float64(data=c.rv))
        self.pub_hl.publish(Float64(data=c.hl))
        self.pub_hr.publish(Float64(data=c.hr))


def _ask(p, v, d=None):
    while True:
        r = input(p).strip()
        if r == '' and d is not None: return d
        try:
            n = int(r)
            if n in v: return n
        except ValueError: pass


def main():
    print("=" * 50)
    print("  AUV Autopilot v50.22")
    print("=" * 50)
    mm = _ask("Двигатели [1=синхрон / 2=дифференциал] (2): ", {1, 2}, 2)
    motor = MotorMode.SINGLE if mm == 1 else MotorMode.DUAL
    if motor == MotorMode.SINGLE: print("  ⚠  Оба движка синхронно, курс через вертик. руль.")
    dm = _ask("Глубина [1=рули / 2=балласты / 3=оба] (1): ", {1, 2, 3}, 1)
    depth = {1: DepthMode.RUDDER, 2: DepthMode.BALLAST, 3: DepthMode.BOTH}[dm]
    print("\n  → Двигатели: " + motor.name + "  |  Глубина: " + depth.name)
    wps = []; print("\nWaypoints (X Y Z). Пустая = старт."); i = 1
    while True:
        try:
            inp = input("  Точка " + str(i) + ": ").strip()
            if not inp:
                if wps: break
                continue
            p = inp.split()
            if len(p) == 3: wps.append(tuple(map(float, p))); i += 1
        except ValueError: pass
    rclpy.init()
    node = AUVAutopilotNode(wps, motor, depth)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit): node._pub(ActuatorCommands())
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
