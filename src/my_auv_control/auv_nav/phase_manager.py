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
        self._aligned = False   # фаза 3: наведён ли нос на точку (с гистерезисом)
        self.seg_start = [0.0, 0.0, 0.0]   # A — начало текущего отрезка для LOS

    def init_wp(self, t, pos):
        raw = list(self.waypoints[self.wp_idx])
        self.target = list(raw)
        # A для LOS: предыдущая путевая точка, если она есть, иначе текущая поза.
        # Так линия маршрута идёт «от точки к точке», а не от случайного места
        # сноса — аппарат держит именно ребро ломаной траектории.
        if self.wp_idx > 0:
            self.seg_start = list(self.waypoints[self.wp_idx - 1])
        else:
            self.seg_start = list(pos)
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
        """Автомат фаз:
          1 NAV        — круиз к XY (далеко, dist >= r_app)
          2 Z_CORRIDOR — Z достигнут раньше XY: держим коридор и едем дальше (2a)
          2 Z_STAB     — в зоне XY (dist < r_app), но Z не доведён: правим Z (2b)
          3 APPROACH   — Z в норме и dist < r_app: стоп → разворот на месте → газ прямо
          4 HOVER_STAB — XY и Z достигнуты
        """
        z_ok = abs(s.z_err) < L.suc_r            # финальная норма по Z (0.5)
        xy_ok = s.dist_2d < L.suc_r              # финальная норма по XY (0.5)
        z_in_corr = abs(s.z_err) <= L.z_corr_in  # вход в коридор по Z (0.4)
        near = s.dist_2d < L.r_app               # зона сближения (3 м)

        # Финал: и XY, и Z в норме
        if z_ok and xy_ok:
            self.state = 'HOVER_STAB'
            return self.state

        st = self.state

        # Коридор по Z (правка глубины НА ХОДУ) имеет смысл только для РУЛЕЙ —
        # им нужен набегающий поток. Для БАЛЛАСТА поток не нужен, и менять глубину
        # на ходу нельзя (он погружался бы, не дойдя до точки) -> входим в фазу Z
        # СТРОГО над/под точкой XY (через Z_STAB, когда near).
        corridor_ok = (self.dm == DepthMode.RUDDER)

        # ---- из NAV ----
        if st == 'NAV':
            # ФАЗА 2 (Z_STAB) входит, когда X,Y СОВПАЛИ (xy_ok), а не просто
            # «подошли на 3 м». Так аппарат сначала ПОЛНОСТЬЮ доводит XY (не
            # «не доплывает»), а Z нагоняется уже во 2-й фазе над точкой.
            if xy_ok and not z_ok:
                self.state = 'Z_STAB'        # 2b: XY совпал, доводим Z
            elif near and z_ok:
                self.state = 'APPROACH'      # Z уже готов, рядом → точное сближение по XY
            elif z_in_corr and corridor_ok:
                self.state = 'Z_CORRIDOR'    # 2a: ТОЛЬКО рули — Z поймали раньше XY

        # ---- из Z_CORRIDOR (2a) ----
        elif st == 'Z_CORRIDOR':
            if xy_ok and not z_ok:
                self.state = 'Z_STAB'        # XY совпал, доводим Z
            elif near and z_ok:
                self.state = 'APPROACH'
            elif abs(s.z_err) > L.z_corr:
                self.state = 'NAV'           # выпали из коридора → восстановить Z

        # ---- из Z_STAB (2b): XY совпал, нагоняем Z ----
        elif st == 'Z_STAB':
            if s.dist_2d > L.r_app:
                self.state = 'NAV'           # XY СИЛЬНО уплыл (>3 м) → снова круиз
            elif z_ok and not xy_ok:
                self.state = 'APPROACH'      # Z доведён, осталось точно закрыть XY
            # z_ok и xy_ok → HOVER_STAB (через общий верхний if)

        # ---- из APPROACH (3): точное сближение по XY (Z норм) ----
        elif st == 'APPROACH':
            # ВАЖНО: НЕ откидываем фазу 3 назад при достижении цели/мелких
            # колебаниях. Возврат только при ЗНАЧИТЕЛЬНОЙ потере (с запасом):
            if abs(s.z_err) > 2.0 * L.z_corr:     # Z реально потерян (>1.0 м)
                self.state = 'Z_STAB' if xy_ok else 'NAV'
            elif s.dist_2d > 1.5 * L.r_app:       # утащило далеко по XY (>4.5 м)
                self.state = 'NAV'

        if self.state != st:
            self.need_pid_reset = True
            self._aligned = False   # при смене фазы сбрасываем наведение

        return self.state

    def params(self, s: VehicleState) -> Dict:
        p = {'bs': 0.0, 'yd': 0.0}
        ra = s.roll_abs
        dist = s.dist_2d

        if self.state == 'NAV':
            import math as _m
            # POINT-AND-SHOOT: если нос сильно отвёрнут (>turn_first) — сперва
            # разворот НА МЕСТЕ полным дифференциалом, ход=0. Иначе аппарат шёл бы
            # вперёд во время поворота и описывал дугу/«катет» (неэффективный путь).
            if abs(s.yaw_err) > L.turn_first:
                # Разворот на месте. Если есть остаточный ход вперёд (s.vel<0) —
                # АКТИВНО ТОРМОЗИМ (реверс), иначе аппарат глиссирует и наматывает дугу.
                p['bs'] = max(0.0, -3.0 * s.vel)   # s.vel<0 (вперёд) -> bs>0 (реверс-тормоз)
                yd = L.turn_gain * s.yaw_err - L.turn_damp * s.yaw_d
                p['yd'] = max(-L.turn_thrust, min(L.turn_thrust, yd))
            else:
                sc = min(1.0, dist / 15.0)
                t = L.cruise * sc
                if ra > 0.15: t *= 0.55
                align = max(0.0, _m.cos(s.yaw_err))
                t *= (0.1 + 0.9 * align * align)   # резче гасим ход при отклонении курса
                if -t < L.cruise_min:
                    t = -L.cruise_min
                p['bs'] = t
                # Дифференциал: при малой ошибке (<5°) — ноль, руль сам выправит.
                # Иначе аппарат дугой идёт вместо прямой.
                if abs(s.yaw_err) < 0.087:          # ~5° — руль справится
                    p['yd'] = 0.0
                else:
                    yd = 6.0 * s.yaw_err
                    ylim = L.yd_frac * abs(t)
                    p['yd'] = max(-ylim, min(ylim, yd))

        elif self.state == 'Z_CORRIDOR':
            # Альт. фаза 2: идём к XY, держим Z в коридоре. Тоже point-and-shoot.
            import math as _m
            if abs(s.yaw_err) > L.turn_first:
                # Разворот на месте. Если есть остаточный ход вперёд (s.vel<0) —
                # АКТИВНО ТОРМОЗИМ (реверс), иначе аппарат глиссирует и наматывает дугу.
                p['bs'] = max(0.0, -3.0 * s.vel)   # s.vel<0 (вперёд) -> bs>0 (реверс-тормоз)
                yd = L.turn_gain * s.yaw_err - L.turn_damp * s.yaw_d
                p['yd'] = max(-L.turn_thrust, min(L.turn_thrust, yd))
            else:
                sc = min(1.0, dist / 15.0)
                t = L.cruise * sc
                if ra > 0.15: t *= 0.55
                align = max(0.0, _m.cos(s.yaw_err))
                t *= (0.1 + 0.9 * align * align)
                z_margin = max(0.0, min(1.0, (L.z_corr - abs(s.z_err)) / max(1e-3, L.z_corr)))
                t *= (0.5 + 0.5 * z_margin)
                if -t < L.cruise_min:
                    t = -L.cruise_min
                p['bs'] = t
                if abs(s.yaw_err) < 0.087:
                    p['yd'] = 0.0
                else:
                    yd = 6.0 * s.yaw_err
                    ylim = L.yd_frac * abs(t)
                    p['yd'] = max(-ylim, min(ylim, yd))

        elif self.state == 'Z_STAB':
            # Фаза 2b: в зоне XY, доводим Z (подъём/спуск)
            if self.dm == DepthMode.RUDDER:
                # ВАЖНО: корпус слегка положительно плавуч (+6.5 Н). Чтобы рули
                # СМОГЛИ погрузить аппарат, им нужен поток v>=~0.45 м/с, т.е. тяга ~12-15,
                # а не -4 (при -4 рули давали лишь 2.6 Н < 6.5 Н и спуск был невозможен).
                # Газуем тем сильнее, чем больше ошибка по Z.
                base = -12.0
                if abs(s.z_err) > 1.0:
                    base = -15.0
                p['bs'] = base
                dc = self.get_drift_corr(s)
                p['yd'] = min(4.0, max(-4.0, 4.0 * (s.yaw_err + dc)))
            else:
                # БАЛЛАСТЫ: НЕ стоп. Балласт меняет глубину независимо от потока,
                # поэтому продолжаем малым ходом ДОВОДИТЬ XY до точки (иначе
                # аппарат вставал в r_app=3 м и «не доплывал» по X/Y), а глубину
                # параллельно тянет балластный контур.
                import math as _m
                if abs(s.yaw_err) > L.turn_first:
                    # сильно отвёрнут — доворот на месте (point-and-shoot)
                    p['bs'] = max(0.0, -3.0 * s.vel)
                    yd = L.turn_gain * s.yaw_err - L.turn_damp * s.yaw_d
                    p['yd'] = max(-L.turn_thrust, min(L.turn_thrust, yd))
                else:
                    # малый ход к точке, плавно слабее у самой цели
                    sc = max(0.3, min(1.0, dist / L.r_app))
                    t = -L.approach_thrust * sc
                    p['bs'] = t
                    yd = 4.0 * s.yaw_err
                    ylim = L.yd_frac * abs(t)
                    p['yd'] = max(-ylim, min(ylim, yd))

        elif self.state == 'APPROACH':
            # Фаза 3: стоп → разворот носом на точку → газ по прямой.
            # Гистерезис наведения: «навёлся» при |ye|<align_tol (±10°),
            # «сбился» только если ушли за align_tol_out (±20°) — без дребезга.
            ye = s.yaw_err
            if self._aligned:
                if abs(ye) > L.align_tol_out:
                    self._aligned = False
            else:
                if abs(ye) < L.align_tol:
                    self._aligned = True

            if not self._aligned:
                # РАЗВОРОТ НА МЕСТЕ: ход=0, ПОЛНЫЙ дифференциал движков (враздрай).
                # PD: P доворачивает быстро, D (по гироскопу yaw_d) гасит занос —
                # без D был чистый P → перелёт курса → вечная осцилляция.
                p['bs'] = 0.0
                yd = L.turn_gain * ye - L.turn_damp * s.yaw_d
                p['yd'] = max(-L.turn_thrust, min(L.turn_thrust, yd))
            else:
                # ГАЗ ПО ПРЯМОЙ с учётом ИНЕРЦИИ: тормозим заранее.
                # s.vel < 0 = ход вперёд (ось X). Тормозной путь ~ v^2/(2a).
                v_fwd = -s.vel                      # скорость вперёд (>0)
                decel = 0.6                         # оценка доступного торможения, м/с^2
                brake_dist = (v_fwd * v_fwd) / (2.0 * decel) if v_fwd > 0 else 0.0
                if s.dist_2d <= brake_dist:
                    # уже пора гасить инерцию → РЕВЕРС тяги (тормозим винтами)
                    p['bs'] = +min(L.approach_thrust, 0.5 * L.cruise_min)
                else:
                    # газ к точке, плавно слабее у самой цели
                    sc = max(0.35, min(1.0, s.dist_2d / L.r_app))
                    p['bs'] = -L.approach_thrust * sc
                yd = 3.0 * ye - 0.5 * s.yaw_d
                ylim = L.yd_frac * L.approach_thrust
                p['yd'] = max(-ylim, min(ylim, yd))

        # HOVER: нули
        return p
