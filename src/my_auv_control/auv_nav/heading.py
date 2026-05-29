"""Heading control — vertical rudder (v51.0). Always active."""
from .models import VehicleState, PID as P, Lim as L


class HeadingController:
    def __init__(self):
        self._rv = 0.0

    @staticmethod
    def _sl(c, t, r, dt):
        d = r * dt
        return c + max(-d, min(d, t - c))

    def compute(self, s: VehicleState, phase: str, dt: float, backoff: bool) -> float:
        if phase in ('HOVER_STAB', 'FINISH'):
            self._rv = self._sl(self._rv, 0.0, L.rud_spd, dt)
            return self._rv
        if backoff:
            tgt = 0.0
        else:
            tgt = P.Kp_yaw * s.yaw_err + P.Kd_yaw * s.yaw_d
            if s.roll_abs > 0.18 and phase == 'XY_FINAL':
                tgt *= 0.35
            tgt = max(-0.5, min(0.5, tgt))
        self._rv = self._sl(self._rv, tgt, L.rud_spd, dt)
        return self._rv

    def reset(self):
        self._rv = 0.0
