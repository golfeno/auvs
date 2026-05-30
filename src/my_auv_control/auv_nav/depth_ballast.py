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
        self._vol = P.bz_neutral  # инициализация с нейтрали (0.493), иначе всплывает
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

        # ── МЁРТВАЯ ЗОНА у цели: близко по Z и почти не движемся вертикально ->
        # держим текущий объём. Иначе насос постоянно подруливал и аппарат
        # слегка осциллировал по глубине у точки. ──
        if abs(s.z_err) < P.bz_deadband and abs(s.dz_dt) < 0.03:
            self._iz *= 0.97
            return self._vol

        # ── КАСКАД с ОГРАНИЧЕНИЕМ СКОРОСТИ ──
        # Внешний контур: ошибка глубины -> ЖЕЛАЕМАЯ верт. скорость Vz_des,
        # ограниченная bz_vz_max (чтобы трение успевало гасить инерцию).
        #   z_err>0 (выше цели) -> надо ВНИЗ -> Vz_des < 0 (Vz: вверх +).
        vz_des = -P.bz_kp_z * s.z_err
        vz_des = max(-P.bz_vz_max, min(P.bz_vz_max, vz_des))

        # Внутренний контур: гоним фактическую Vz (s.dz_dt) к желаемой.
        # Ошибка скорости -> отклонение объёма от нейтрали.
        #   нужна бОльшая Vz вверх -> больше плавучести -> volume вверх.
        vz_err = vz_des - s.dz_dt
        adj = P.bz_kp_v * vz_err

        # интеграл по ошибке скорости (убирает статич. остаток у цели)
        self._iz += vz_err * dt
        self._iz = max(-P.bz_ilim, min(P.bz_ilim, self._iz))
        if abs(s.z_err) < 0.2:
            self._iz *= 0.95
        adj += P.bz_ki_v * self._iz

        # ОГРАНИЧЕНИЕ АВТОРИТЕТА (силы): |отклонение| <= bz_authority
        adj = max(-P.bz_authority, min(P.bz_authority, adj))
        target = P.bz_neutral + adj

        # SLEW: объём не быстрее насоса
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
