"""Ballast-based depth control (v51.0).

Uses BuoyancyEngine to change ballast volume → changes buoyancy → changes depth.

PID: depth error → ballast volume (0..1)
z_err = pos.z - target.z, pos.z — высота в мире (вверх +).
- z_err > 0 (ВЫШЕ цели) → надо ВНИЗ → МЕНЬШЕ плавучести → volume DOWN
- z_err < 0 (НИЖЕ цели) → надо ВВЕРХ → БОЛЬШЕ плавучести → volume UP

Pitch trim: differential ballast between bow and stern.
"""
from .models import VehicleState, PID as P, Phys


class DepthBallastController:
    def __init__(self):
        self._vol = 0.5      # normalized 0..1
        self._iz = 0.0
        self._pitch_iz = 0.0

    def reset(self):
        self._iz = 0.0
        self._pitch_iz = 0.0

    def compute(self, s: VehicleState, phase: str, dt: float) -> float:
        """Returns normalized ballast volume (0..1).
        0 = empty (less buoyancy), 1 = full (more buoyancy).
        Neutral ≈ 0.5."""
        if phase in ('HOVER_STAB', 'FINISH'):
            # Hold current volume
            return self._vol

        # ── Depth PID → volume ──
        self._iz += s.z_err * dt
        self._iz = max(-P.bz_ilim, min(P.bz_ilim, self._iz))
        if abs(s.z_err) < 0.2:
            self._iz *= 0.95

        # Predictive braking for Z
        Kd = P.Kd_bz
        v_z = abs(s.dz_dt)
        if v_z > 0.05 and abs(s.z_err) < 2.0:
            decel = max(0.5, 2.0 * abs(s.vel))
            brake_dist = (v_z * v_z) / (2.0 * decel)
            if abs(s.z_err) < brake_dist * 1.5:
                Kd *= 2.0

        # PID (знак минус): z_err>0 (выше цели) → меньше плавучести → volume вниз
        adj = -(P.Kp_bz * s.z_err + P.Ki_bz * self._iz + Kd * s.dz_dt)

        # ОГРАНИЧЕНИЕ АВТОРИТЕТА: |отклонение от нейтрали| <= bz_authority,
        # иначе балласт даёт ±2 м/с² и аппарат 'взлетает'. Ограничиваем силу.
        adj = max(-P.bz_authority, min(P.bz_authority, adj))
        target = P.bz_neutral + adj

        # SLEW: объём не может меняться быстрее насоса (bz_slew norm/с)
        d = P.bz_slew * dt
        self._vol += max(-d, min(d, target - self._vol))
        self._vol = max(0.0, min(1.0, self._vol))
        return self._vol

    def get_pitch_trim(self, s: VehicleState, dt: float) -> float:
        """Returns differential volume for pitch trim (positive = bow heavier).
        Applied as: bow_volume = base + trim, stern_volume = base - trim."""
        pitch_target = 0.0
        pitch_err = s.pitch_curr - pitch_target

        self._pitch_iz += pitch_err * dt
        self._pitch_iz = max(-0.5, min(0.5, self._pitch_iz))
        if abs(pitch_err) < 0.05:
            self._pitch_iz *= 0.95

        trim = 0.3 * pitch_err + 0.02 * self._pitch_iz + 0.5 * s.pitch_d
        return max(-0.2, min(0.2, trim))
