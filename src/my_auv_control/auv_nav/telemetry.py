"""Telemetry (v54.0) — строка состояния + источники ориентации (IMU/Odo/Kalman) + датчики."""
import sys, math
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from .models import VehicleState, PHASE_TR, ActuatorCommands


class Telemetry:
    def __init__(self, node):
        self.n = node
        self.pub_pos = node.create_publisher(Point, '/auv/position', 10)
        self.pub_status = node.create_publisher(String, '/auv/status', 10)
        self._t = 0.0
        self._pv = 0.0; self._pz = 0.0
        self._ax = 0.0; self._az = 0.0

    def analytics(self, s, phase, wp_idx):
        try:
            p = Point()
            p.x = float(s.pos[0]); p.y = float(s.pos[1]); p.z = float(s.pos[2])
            self.pub_pos.publish(p)
            m = String()
            m.data = str(wp_idx+1) + "_" + str(phase)
            self.pub_status.publish(m)
        except Exception:
            pass

    def log(self, s, phase, wp_idx, t, tw, cmd=None):
        if t - self._t < 0.15:
            return
        dt = t - self._t if self._t > 0 else 0.15
        self._t = t
        self._ax = (s.vel - self._pv) / dt if dt > 0.01 else 0.0
        self._az = (s.dz_dt - self._pz) / dt if dt > 0.01 else 0.0
        self._pv = s.vel; self._pz = s.dz_dt

        ru = PHASE_TR.get(phase, str(phase))
        # СЛИТАЯ оценка (используется регуляторами)
        rd = math.degrees(s.rpy[0]);  pd = math.degrees(s.rpy[1]);  yd = math.degrees(s.rpy[2])
        # отдельные источники
        ri, pi_, yi = (math.degrees(a) for a in s.rpy_imu)
        ro, po, yo = (math.degrees(a) for a in s.rpy_odo)
        src = "KALMAN" if s.imu_ok else "ODO"

        # Режим обхода: NORMAL / AVOID-L/R/U/D / WALL_FOLLOW
        am = getattr(s, 'avoid_mode', 'NORMAL')
        avoid = '' if am == 'NORMAL' else f"[ОБХОД:{am}] "

        son = f"{s.sonar_fwd:.1f}м" if (s.sonar_ok and s.sonar_fwd >= 0) else "чисто"
        # Диагностика обхода: ближайшая точка / валидные лучи / выбор стороны / поправка курса
        def _zf(v):  # 999 (нет эха) -> "∞"
            return "∞" if v >= 900 else f"{v:.1f}"
        dbg = (f" | СОНАР[лучи:{getattr(s, 'sonar_valid', 0)} "
               f"бл:{getattr(s, 'avoid_closest', 999):.1f}м "
               f"L{_zf(getattr(s,'zL',999))} R{_zf(getattr(s,'zR',999))} "
               f"U{_zf(getattr(s,'zU',999))} D{_zf(getattr(s,'zD',999))} "
               f"dir:{getattr(s, 'avoid_dir', '-')} "
               f"yaw_av:{math.degrees(getattr(s, 'avoid_yaw', 0.0)):+.0f}°]")
        alt = f"{s.alt_floor:.1f}м" if s.alt_floor >= 0 else "--"
        dl = getattr(self, 'depth_label', '')   # режим глубины (ставит autopilot)

        line = (
            f"WP {wp_idx+1}/{tw} {ru:<9s} "
            f"X:{s.pos[0]:+6.1f} Y:{s.pos[1]:+6.1f} Z:{s.pos[2]:+5.2f} "
            f"V:{s.vel:+.2f} Vz:{s.dz_dt:+.2f} "
            f"D:{s.dist_2d:4.1f}м dZ:{s.z_err:+.2f} "
            f"{avoid}"
            f"RPY:{rd:+.0f}/{pd:+.0f}/{yd:+.0f}° Сонар:{son}"
        )

        line += dbg
        # Состояние сенсоров: источник ориентации, магнитометр, альтиметр (дно).
        mag = f"{math.degrees(s.mag_heading):+.0f}°" if getattr(s, 'mag_ok', False) else "нет"
        line += (f" | СЕНС[ор:{src} mag:{mag} дно:{alt} "
                 f"имю:{'да' if s.imu_ok else 'нет'} "
                 f"сонар:{'да' if s.sonar_ok else 'нет'}]")
        if dl:
            line += f" | {dl}"
        if cmd is not None:
            rv = math.degrees(getattr(cmd, 'rv', 0.0))
            hl = math.degrees(getattr(cmd, 'hl', 0.0))
            hr = math.degrees(getattr(cmd, 'hr', 0.0))
            line += f" | руль в:{rv:+.0f}° гор:{hl:+.0f}/{hr:+.0f}°"
            if hasattr(cmd, 'ballast_volume') and cmd.ballast_volume != 0.5:
                line += f" B:{cmd.ballast_volume:.0%}"

        # ОДНА перезаписываемая строка: \r в начало + \033[K очистка хвоста,
        # БЕЗ переноса -> новая публикация НЕ создаёт новую строку.
        sys.stdout.write("\r\033[K" + line)
        sys.stdout.flush()

    def log_wp(self, wi):
        sys.stdout.write("\n  ✓ Точка " + str(wi) + "\n")
        sys.stdout.flush()
