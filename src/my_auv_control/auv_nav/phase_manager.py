"""Phase Manager (v52.0) — две альтернативные «фазы 2».

NAV: круиз к точке по XY (Z вторична, грубо)
ФАЗА 2 (одна из двух, по ситуации):
  • Z_STAB     — XY достигнут раньше Z → встаём/тормозим и доводим Z
                   RUDDER: малый ход вперёд + рули; BALLAST: стоп + балласты
  • Z_CORRIDOR — Z достигнут раньше XY → продолжаем идти к XY, удерживая
                   Z в коридоре |z_err| <= 0.5 м (альт. версия фазы 2)
HOVER: и XY, и Z в норме
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
        z_ok = abs(s.z_err) < L.suc_r          # 0.5 — финальная норма по Z
        xy_ok = s.dist_2d < L.suc_r            # 0.5 — норма по XY
        z_in_corr = abs(s.z_err) <= L.z_corr_in  # 0.4 — вход в коридор по Z

        # Оба ОК → HOVER
        if z_ok and xy_ok:
            self.state = 'HOVER_STAB'
            return self.state

        # NAV → фаза 2 (две альтернативы):
        if self.state == 'NAV' and xy_ok and not z_ok:
            # XY достигнут раньше Z → классический Z_STAB
            self.state = 'Z_STAB'
            self.need_pid_reset = True
        elif self.state == 'NAV' and z_in_corr and not xy_ok:
            # Z достигнут раньше XY → коридор по Z (альт. фаза 2)
            self.state = 'Z_CORRIDOR'
            self.need_pid_reset = True

        # Переходы из коридора
        if self.state == 'Z_CORRIDOR':
            if xy_ok and not z_ok:
                # доехали по XY, но Z вне нормы → доводим Z на месте
                self.state = 'Z_STAB'
                self.need_pid_reset = True
            elif abs(s.z_err) > L.z_corr:
                # вылетели из коридора по Z → назад в NAV (восстановить Z)
                self.state = 'NAV'
                self.need_pid_reset = True

        # Z_STAB → NAV: если Z попали, но XY уехал (дрифт)
        if self.state == 'Z_STAB' and not xy_ok:
            self.state = 'NAV'
            self.need_pid_reset = True

        return self.state

    def params(self, s: VehicleState) -> Dict:
        p = {'bs': 0.0, 'yd': 0.0}
        ra = s.roll_abs
        dist = s.dist_2d

        if self.state == 'NAV':
            # Едем к точке. Тяга масштабируется с дистанцией, но НЕ гаснет в ноль:
            # держим минимальный ход, иначе рули теряют поток у самой точки.
            sc = min(1.0, dist / 15.0)
            t = L.cruise * sc
            if ra > 0.15: t *= 0.55
            # taper по курсу: пока нос не наведён на точку — сбавляем ход,
            # чтобы доворачивать на месте, а не описывать круги вокруг цели
            import math as _m
            align = max(0.0, _m.cos(s.yaw_err))
            t *= (0.25 + 0.75 * align)
            # пол по модулю тяги (L.cruise отрицательна → ход вперёд)
            if -t < L.cruise_min:
                t = -L.cruise_min
            p['bs'] = t
            # дифференциал курса ограничиваем долей тяги, чтобы моторы не реверсировали
            # друг против друга (иначе аппарат крутится на месте вместо подхода)
            yd = 5.0 * s.yaw_err
            ylim = L.yd_frac * abs(t)
            p['yd'] = max(-ylim, min(ylim, yd))

        elif self.state == 'Z_CORRIDOR':
            # Альт. фаза 2: продолжаем идти к XY, удерживая Z в коридоре.
            sc = min(1.0, dist / 15.0)
            t = L.cruise * sc
            if ra > 0.15: t *= 0.55
            import math as _m
            align = max(0.0, _m.cos(s.yaw_err))
            t *= (0.25 + 0.75 * align)
            # мягче у края коридора (даём глубине отработать), но не ниже мин. хода
            z_margin = max(0.0, min(1.0, (L.z_corr - abs(s.z_err)) / max(1e-3, L.z_corr)))
            t *= (0.5 + 0.5 * z_margin)
            if -t < L.cruise_min:
                t = -L.cruise_min
            p['bs'] = t
            yd = 5.0 * s.yaw_err
            ylim = L.yd_frac * abs(t)
            p['yd'] = max(-ylim, min(ylim, yd))

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
