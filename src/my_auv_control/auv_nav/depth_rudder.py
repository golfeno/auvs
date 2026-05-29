"""Rudder-based depth control (v51.0).

PID cascaded: depth error → pitch command → rudder deflection.
Sign: z_err > 0 = too deep → need nose UP → rudder deflects to push stern down.
    (stern rudder: opposite direction from bow rudder)
"""
from .models import VehicleState, PID as P, Lim as L


class DepthRudderController:
    def __init__(self):
        self._hl = 0.0
        self._hr = 0.0
        self._iz = 0.0  # integral for depth

    @staticmethod
    def _sl(c, t, r, dt):
        d = r * dt
        return c + max(-d, min(d, t - c))

    def reset(self):
        self._iz = 0.0

    def compute(self, s: VehicleState, phase: str, dt: float):
        """Returns (hl, hr) — horizontal rudder left/right positions."""
        vel_abs = max(0.1, abs(s.vel))
        vs = max(0.3, min(1.0, vel_abs / 2.0))

        Kp = P.Kp_z * vs
        Ki = P.Ki_z * vs
        Kd = P.Kd_z * vs

        # Integral with anti-windup
        self._iz += s.z_err * dt
        self._iz = max(-P.z_ilim, min(P.z_ilim, self._iz))
        if abs(s.z_err) < 0.2:
            self._iz *= 0.95

        # Predictive braking: increase Kd near target
        v_z = abs(s.dz_dt)
        if v_z > 0.05 and abs(s.z_err) < 2.0:
            decel = max(0.3, 3.0 * vel_abs)
            brake_dist = (v_z * v_z) / (2.0 * decel)
            if abs(s.z_err) < brake_dist * 1.5:
                Kd *= 2.0

        # PID output
        # Sign: z_err > 0 = too deep → need nose up → positive rudder (stern down)
        roll_damp = 1.0 - max(0.0, min(0.6, s.roll_abs * 2.0))
        th = (Kp * s.z_err + Ki * self._iz + Kd * s.dz_dt) * roll_damp

        # Soft pitch limit
        max_pitch = 0.30
        if s.pitch_curr > max_pitch and th > 0:
            th = min(th, 0.1)
        elif s.pitch_curr < -max_pitch and th < 0:
            th = max(th, -0.1)

        th = max(-0.55, min(0.55, th))

        # Roll stabilisation
        rp = P.Kp_roll * s.rpy[0] + P.Kd_roll * s.roll_d
        rp *= max(0.15, 1.0 - (abs(s.z_err) / 5.0))

        raw_hl = max(-0.95, min(0.95, th - rp - P.roll_bias))
        raw_hr = max(-0.95, min(0.95, th + rp + P.roll_bias))

        self._hl = self._sl(self._hl, raw_hl, L.rud_spd, dt)
        self._hr = self._sl(self._hr, raw_hr, L.rud_spd, dt)
        return self._hl, self._hr
