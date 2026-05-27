"""Phase Manager (v50.26) — parallel Z+XY, BRAKE→TURN→GO, radius 0.5."""
from typing import List, Tuple, Dict
from .models import VehicleState, Lim as L, MotorMode, Phys
import math


class PhaseManager:
    Z_OFFSET = 0.4
    MIN_SPEED = -3.0
    Z_OK = 0.6
    XY_OK = 0.5        # ← 0.85 → 0.5
    Z_CRIT = 1.5

    def __init__(self, wps, mm=MotorMode.DUAL):
        self.waypoints = wps; self.wp_idx = 0; self.state = 'INIT'; self.mm = mm
        self.target = list(wps[0])
        self.need_pid_reset = False; self.restab = False
        self.xy_sub = 'GO'; self.xy_t = 0.0
        self.drift_vx = 0.0; self.drift_vy = 0.0

    def init_wp(self, t, pos):
        raw = list(self.waypoints[self.wp_idx])
        self.target = [raw[0], raw[1], raw[2] - self.Z_OFFSET]
        self.state = 'NAV'
        self.need_pid_reset = True
        self.xy_sub = 'GO'; self.xy_t = t

    def update_drift(self, s, t, dt):
        if dt <= 0: return
        heading = s.rpy[2]
        vx = s.vel * math.cos(heading)
        vy = s.vel * math.sin(heading)
        a = 0.05
        self.drift_vx = (1-a)*self.drift_vx + a*vx
        self.drift_vy = (1-a)*self.drift_vy + a*vy

    def get_drift_correction(self, s):
        drift_perp = (-math.sin(s.bearing)*self.drift_vx +
                       math.cos(s.bearing)*self.drift_vy)
        dist = max(0.5, s.dist_2d)
        corr = math.atan2(drift_perp*2.0, abs(s.vel)+0.5)
        return max(-0.3, min(0.3, corr/dist))

    def evaluate(self, s: VehicleState, t: float) -> str:
        z_bad = abs(s.z_err) > self.Z_OK
        xy_bad = s.dist_2d > self.XY_OK
        z_critical = abs(s.z_err) > self.Z_CRIT

        # Both OK → HOVER
        if not z_bad and not xy_bad:
            self.state = 'HOVER_STAB'
            return self.state

        # Both bad → priority
        if z_bad and xy_bad:
            self.state = 'Z_PRIORITY' if z_critical else 'NAV'
        elif z_bad and not xy_bad:
            self.state = 'Z_PRIORITY'
        elif not z_bad and xy_bad:
            # XY approach with BRAKE→TURN→GO
            if s.dist_2d < 2.0:
                self.state = 'XY_FINAL'
                if self.xy_sub == 'GO' and abs(s.vel) > 0.4:
                    self.xy_sub = 'BRAKE'; self.xy_t = t
            else:
                self.state = 'NAV'

        # XY_FINAL sub-phases
        if self.state == 'XY_FINAL':
            if self.xy_sub == 'BRAKE':
                if abs(s.vel) < 0.4:
                    self.xy_sub = 'TURN'; self.xy_t = t
                elif (t - self.xy_t) > 4.0:
                    self.xy_sub = 'TURN'; self.xy_t = t
            elif self.xy_sub == 'TURN':
                if abs(s.yaw_err) < 0.12:
                    self.xy_sub = 'GO'
                elif (t - self.xy_t) > 15.0:
                    self.xy_sub = 'GO'

        return self.state

    def params(self, s: VehicleState) -> Dict:
        p = {'bs': 0.0, 'yd': 0.0}; ra = s.roll_abs

        if self.state == 'NAV':
            scale = min(1.0, s.dist_2d / 15.0)
            t = -35.0 * scale
            if ra > 0.15: t *= 0.55
            p['bs'] = t; p['yd'] = 5.0 * s.yaw_err

        elif self.state == 'Z_PRIORITY':
            # v49.10 speeds
            if self.restab:
                t = -18.0 if abs(s.z_err) > 1.4 else -8.0
            else:
                t = -16.0
            if ra > 0.15: t *= 0.6
            p['bs'] = t
            drift_corr = self.get_drift_correction(s)
            p['yd'] = min(3.0, max(-3.0, 4.0 * (s.yaw_err + drift_corr)))

        elif self.state == 'XY_FINAL':
            if self.xy_sub == 'BRAKE':
                p['bs'] = self.MIN_SPEED; p['yd'] = 0.0
            elif self.xy_sub == 'TURN':
                zf = max(0.5, min(1.5, 1.0 / (abs(s.z_err) + 0.3)))
                tf = 20.0 * zf * s.yaw_err
                if abs(s.yaw_err) > 0.5:
                    tf = max(abs(tf), 12.0) * (1 if s.yaw_err > 0 else -1)
                p['bs'] = self.MIN_SPEED
                p['yd'] = max(-25.0, min(25.0, tf))
            elif self.xy_sub == 'GO':
                dist = s.dist_2d
                if dist > 3.0: t = -10.0
                elif dist > 1.5: t = -6.0
                else: t = -2.0
                if ra > 0.10: t *= 0.5
                p['bs'] = t
                drift_corr = self.get_drift_correction(s)
                p['yd'] = min(4.0, max(-4.0, 5.0 * (s.yaw_err + drift_corr)))

        return p
