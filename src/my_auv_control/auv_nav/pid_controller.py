"""PID Controller (v52.0) — all 6 rudders, dedicated roll channel."""
from .models import (VehicleState, ActuatorCommands, PID as P, Lim as L,
                     SR, MotorMode, DepthMode, Phys)
import math


class PIDController:
    def __init__(self, mm=MotorMode.DUAL, dm=DepthMode.RUDDER):
        self.mm = mm; self.dm = dm
        self._b = 0.0; self._rv = 0.0; self._rvt = 0.0
        self._hl = 0.0; self._hr = 0.0; self._hfl = 0.0; self._hfr = 0.0
        self._iz = 0.0; self._biz = 0.0; self._vol = 0.5

    @staticmethod
    def _sl(c, t, r, dt):
        d = r * dt; return c + max(-d, min(d, t - c))

    def reset(self):
        self._iz = 0.0; self._biz = 0.0; self._rvt = 0.0

    def force_zero_thrust(self):
        self._b = 0.0

    def compute(self, s, phase, tp, dt, bo=False, turning=False):
        a = ActuatorCommands()
        a.rv = self._heading(s, phase, dt, bo)
        a.rvt = self._roll_vertical_top(s, phase, dt)
        if self.dm in (DepthMode.RUDDER, DepthMode.BOTH):
            a.hl, a.hr, a.hfl, a.hfr = self._depth_rudder(s, phase, dt)
        if self.dm in (DepthMode.BALLAST, DepthMode.BOTH):
            a.ballast_volume = self._depth_ballast(s, phase, dt)
        a.lt, a.rt = self._thrust(s, phase, tp, dt, bo)
        return a

    def _heading(self, s, phase, dt, bo):
        if phase in ('HOVER_STAB', 'FINISH'):
            self._rv = self._sl(self._rv, 0.0, L.rud_spd, dt); return self._rv
        if bo: tgt = 0.0
        else:
            tgt = P.Kp_yaw * s.yaw_err + P.Kd_yaw * s.yaw_d
            if s.roll_abs > 0.18 and phase == 'XY_FINAL': tgt *= 0.35
            tgt = max(-0.5, min(0.5, tgt))
        self._rv = self._sl(self._rv, tgt, L.rud_spd, dt); return self._rv

    def _roll_vertical_top(self, s, phase, dt):
        """Верхний вертикальный руль — стабилизация крена. НЕ инвертируется."""
        tgt = P.Kp_roll_v * s.rpy[0] + P.Kd_roll_v * s.roll_d
        tgt = max(-P.roll_v_lim, min(P.roll_v_lim, tgt))
        self._rvt = self._sl(self._rvt, tgt, L.rud_spd, dt)
        return self._rvt

    def _depth_rudder(self, s, phase, dt):
        """Pure PID — no constant bias. Anti-windup integral."""
        va = max(0.1, abs(s.vel))
        vel_scale = max(0.4, min(1.0, va / 2.5))
        Kp_z = P.Kp_z * vel_scale
        Ki_z = P.Ki_z * vel_scale
        Kd_z = P.Kd_z * vel_scale

        # Integral with anti-windup
        self._iz += s.z_err * dt
        self._iz = max(-P.z_ilim, min(P.z_ilim, self._iz))
        if abs(s.z_err) < 0.1:
            self._iz *= 0.95

        # Predictive Z braking
        v_z = abs(s.dz_dt)
        if v_z > 0.05 and abs(s.z_err) < 2.0:
            decel = max(0.3, 3.0 * va)
            brake_dist = (v_z * v_z) / (2.0 * decel)
            if abs(s.z_err) < brake_dist * 1.5:
                Kd_z *= 2.0

        # Pure PID output — sign: positive z_err = too deep → nose up → positive rudder
        roll_damp = 1.0 - max(0.0, min(0.6, s.roll_abs * 2.0))
        tgt_h = -(Kp_z * s.z_err + Ki_z * self._iz + Kd_z * s.dz_dt) * roll_damp

        # Pitch limiting: не даём носу уходить слишком далеко
        max_pitch = 0.30  # ~17 градусов
        if s.pitch_curr > max_pitch and tgt_h > 0:
            tgt_h = min(tgt_h, 0.1)  # ограничиваем
        elif s.pitch_curr < -max_pitch and tgt_h < 0:
            tgt_h = max(tgt_h, -0.1)

        tgt_h = max(-0.55, min(0.55, tgt_h))

        # Roll stabilisation (общий дифференциальный терм для всех рулей)
        rp = P.Kp_roll * s.rpy[0] + P.Kd_roll * s.roll_d
        rp *= max(0.15, 1.0 - (abs(s.z_err) / 5.0))

        # Носовые: глубина зеркальна, крен сонаправлен
        tgt_h_bow = P.frud_depth_sign * tgt_h
        rp_bow = P.frud_roll_sign * rp

        raw_hl  = max(-0.95, min(0.95, tgt_h - rp - P.roll_bias))
        raw_hr  = max(-0.95, min(0.95, tgt_h + rp + P.roll_bias))
        raw_hfl = max(-0.95, min(0.95, tgt_h_bow - rp_bow - P.roll_bias))
        raw_hfr = max(-0.95, min(0.95, tgt_h_bow + rp_bow + P.roll_bias))

        self._hl  = self._sl(self._hl,  raw_hl,  L.rud_spd, dt)
        self._hr  = self._sl(self._hr,  raw_hr,  L.rud_spd, dt)
        self._hfl = self._sl(self._hfl, raw_hfl, L.rud_spd, dt)
        self._hfr = self._sl(self._hfr, raw_hfr, L.rud_spd, dt)
        return self._hl, self._hr, self._hfl, self._hfr

    def _depth_ballast(self, s, phase, dt):
        self._biz += s.z_err * dt
        self._biz = max(-P.bz_ilim, min(P.bz_ilim, self._biz))
        if abs(s.z_err) < 0.15: self._biz *= 0.97
        adj = -(P.Kp_bz * s.z_err + P.Ki_bz * self._biz + P.Kd_bz * s.dz_dt)
        self._vol = max(0.0, min(1.0, Phys.BALLAST_NEUTRAL + adj))
        return self._vol

    def _thrust(self, s, phase, tp, dt, bo):
        slew_map = {'NAV': 15.0, 'Z_PRIORITY': 14.0, 'XY_FINAL': 25.0,
                     'HOVER_STAB': 0.0, 'FINISH': 0.0}
        sl = 18.0 if bo else slew_map.get(phase, 15.0)
        self._b = self._sl(self._b, tp.get('bs', 0.0), sl, dt)
        if phase in ('HOVER_STAB', 'FINISH'): return 0.0, 0.0
        yd = tp.get('yd', 0.0)
        if self.mm == MotorMode.SINGLE: return self._b, self._b
        return self._b + yd, self._b - yd
