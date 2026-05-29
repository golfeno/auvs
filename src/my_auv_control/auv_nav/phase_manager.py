"""Phase Manager (v51.6) — NAV to XY first, then fix Z.

NAV: cruise to waypoint XY (Z is secondary, approximate only)
Z_STAB: at XY position → correct Z
  RUDDER: slow forward + rudder depth
  BALLAST: full stop + ballast depth
HOVER: both XY and Z OK
"""
from typing import List, Tuple, Dict
from .models import VehicleState, Lim as L, MotorMode, DepthMode, Phys
import math


class PhaseManager:
    def __init__(self, wps, mm=MotorMode.DUAL, dm=DepthMode.RUDDER):
        self.waypoints = wps
        self.wp_idx = 0
        self.state = 'INIT'
        self.mm = mm
        self.dm = dm
        self.target = list(wps[0])
        self.need_pid_reset = False
        self.drift_vx = 0.0
        self.drift_vy = 0.0

    def init_wp(self, t, pos):
        raw = list(self.waypoints[self.wp_idx])
        self.target = list(raw)
        self.state = 'NAV'
        self.need_pid_reset = True

    def update_drift(self, s, t, dt):
        if dt <= 0: return
        h = s.rpy[2]
        vx = s.vel * math.cos(h)
        vy = s.vel * math.sin(h)
        a = 0.05
        self.drift_vx = (1 - a) * self.drift_vx + a * vx
        self.drift_vy = (1 - a) * self.drift_vy + a * vy

    def get_drift_corr(self, s):
        dp = (-math.sin(s.bearing) * self.drift_vx +
              math.cos(s.bearing) * self.drift_vy)
        d = max(0.5, s.dist_2d)
        c = math.atan2(dp * 2.0, abs(s.vel) + 0.5)
        return max(-0.3, min(0.3, c / d))

    def evaluate(self, s: VehicleState, t: float) -> str:
        z_ok = abs(s.z_err) < 0.5
        xy_ok = s.dist_2d < L.suc_r

        # Оба ОК → HOVER
        if z_ok and xy_ok:
            self.state = 'HOVER_STAB'
            return self.state

        # NAV → Z_STAB: только когда XY ДОСТИГНУТ
        if self.state == 'NAV' and xy_ok and not z_ok:
            self.state = 'Z_STAB'
            self.need_pid_reset = True

        # Z_STAB → NAV: если Z попали но XY уехал (дрифт)
        if self.state == 'Z_STAB' and not xy_ok:
            self.state = 'NAV'
            self.need_pid_reset = True

        return self.state

    def params(self, s: VehicleState) -> Dict:
        p = {'bs': 0.0, 'yd': 0.0}
        ra = s.roll_abs
        dist = s.dist_2d

        if self.state == 'NAV':
            # Едем к точке, полная скорость
            sc = min(1.0, dist / 15.0)
            t = L.cruise * sc
            if ra > 0.15: t *= 0.55
            p['bs'] = t
            p['yd'] = 5.0 * s.yaw_err

        elif self.state == 'Z_STAB':
            # На месте по XY, корректируем Z
            if self.dm == DepthMode.RUDDER:
                # Рулям нужен поток → малая скорость вперёд
                p['bs'] = -4.0
                dc = self.get_drift_corr(s)
                p['yd'] = min(3.0, max(-3.0, 4.0 * (s.yaw_err + dc)))
            else:
                # Балласты: полный стоп
                p['bs'] = 0.0
                p['yd'] = 0.0

        # HOVER: нули
        return p
