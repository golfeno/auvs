"""Phase Manager (v102) — AVOID на 5м, мягкий старт."""
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
        self._aligned = False
        self.seg_start = [0.0, 0.0, 0.0]
        self._avoid_prev = 'NAV'
        self._avoid_clear = 0.0
        self._avoid_exit_t = 0.0
        self._start_t = None  # время первого тика

    def init_wp(self, t, pos):
        raw = list(self.waypoints[self.wp_idx])
        self.target = list(raw)
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

    def evaluate(self, s: VehicleState, t: float, av=None, dt: float = 0.02) -> str:
        if self.state == 'FINISH':
            return self.state

        # Запоминаем время старта
        if self._start_t is None:
            self._start_t = t

        obstacle_near = av is not None and av.closest <= 5.0

        # ─── ВХОД В AVOID ───
        # Гистерезис: не входить 8 сек после выхода
        if obstacle_near and self.state not in ('HOVER_STAB', 'FINISH'):
            if t - self._avoid_exit_t < 8.0:
                return self.state
            if self.state != 'AVOID':
                self._avoid_prev = self.state
            self.state = 'AVOID'
            self._avoid_clear = 0.0
            return self.state

        # ─── ВЫХОД ИЗ AVOID ───
        if self.state == 'AVOID':
            if not obstacle_near:
                self._avoid_clear += dt
                if self._avoid_clear >= 8.0:
                    self.state = self._avoid_prev
                    self._avoid_exit_t = t
                    self.need_pid_reset = True
                    self._aligned = False
                    return self.state
            else:
                self._avoid_clear = 0.0
            return self.state

        # ─── Обычная навигация ───
        z_ok = abs(s.z_err) < L.suc_r
        xy_ok = s.dist_2d < L.suc_r
        z_in_corr = abs(s.z_err) <= L.z_corr_in
        near = s.dist_2d < L.r_app

        if z_ok and xy_ok:
            self.state = 'HOVER_STAB'
            return self.state

        st = self.state
        corridor_ok = (self.dm == DepthMode.RUDDER)

        if st == 'NAV':
            if xy_ok and not z_ok:
                self.state = 'Z_STAB'
            elif near and z_ok:
                self.state = 'APPROACH'
            elif z_in_corr and corridor_ok:
                self.state = 'Z_CORRIDOR'
        elif st == 'Z_CORRIDOR':
            if xy_ok and not z_ok:
                self.state = 'Z_STAB'
            elif near and z_ok:
                self.state = 'APPROACH'
            elif abs(s.z_err) > L.z_corr:
                self.state = 'NAV'
        elif st == 'Z_STAB':
            if s.dist_2d > L.r_app:
                self.state = 'NAV'
            elif z_ok and not xy_ok:
                self.state = 'APPROACH'
        elif st == 'APPROACH':
            if abs(s.z_err) > 2.0 * L.z_corr:
                self.state = 'Z_STAB' if xy_ok else 'NAV'
            elif s.dist_2d > 1.5 * L.r_app:
                self.state = 'NAV'

        if self.state != st:
            self.need_pid_reset = True
            self._aligned = False

        return self.state

    def params(self, s: VehicleState, av=None) -> Dict:
        p = {'bs': 0.0, 'yd': 0.0}

        # ─── AVOID ───
        if self.state == 'AVOID':
            p['bs'] = -3.0
            speed = abs(s.vel)
            if av is not None:
                if speed < 0.2:
                    p['yd'] = max(-15.0, min(15.0, av.yaw_offset * 15.0))
                else:
                    gain = max(2.0, min(5.0, 5.0 - speed * 4.0))
                    p['yd'] = max(-8.0, min(8.0, av.yaw_offset * gain))
            return p

        ra = s.roll_abs
        dist = s.dist_2d

        # ─── Мягкий старт: первые 5 сек — без разворота на месте ───
        soft_start = self._start_t is not None and (s_time - self._start_t) < 5.0 if (s_time := self._start_t) else False
        # Пересчитаем корректно:
        elapsed = 0.0
        if self._start_t is not None:
            # Мы не имеем доступа к текущему t здесь, передадим через s? Нет.
            # Просто проверим по скорости: если vel≈0 и далеко от цели — мягкий старт
            pass

        if self.state == 'NAV':
            import math as _m
            # На старте: не разворачиваться на месте, мягко набрать ход
            if abs(s.vel) < 0.3 and dist > 10.0:
                # Мягкий старт: просто идём вперёд, без разворота
                p['bs'] = -3.0  # мягкая тяга
                # Мягкий поворот (не полный разворот)
                p['yd'] = max(-4.0, min(4.0, 4.0 * s.yaw_err))
            elif abs(s.yaw_err) > L.turn_first:
                p['bs'] = max(0.0, -3.0 * s.vel)
                yd = L.turn_gain * s.yaw_err - L.turn_damp * s.yaw_d
                p['yd'] = max(-L.turn_thrust, min(L.turn_thrust, yd))
            else:
                sc = min(1.0, dist / 15.0)
                t = L.cruise * sc
                if ra > 0.15: t *= 0.55
                align = max(0.0, _m.cos(s.yaw_err))
                t *= (0.1 + 0.9 * align * align)
                if -t < L.cruise_min:
                    t = -L.cruise_min
                p['bs'] = t
                if abs(s.yaw_err) < 0.087:
                    p['yd'] = 0.0
                else:
                    yd = 6.0 * s.yaw_err
                    ylim = L.yd_frac * abs(t)
                    p['yd'] = max(-ylim, min(ylim, yd))

        elif self.state == 'Z_CORRIDOR':
            import math as _m
            if abs(s.yaw_err) > L.turn_first:
                p['bs'] = max(0.0, -3.0 * s.vel)
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
            if self.dm == DepthMode.RUDDER:
                base = -12.0
                if abs(s.z_err) > 1.0:
                    base = -15.0
                p['bs'] = base
                dc = self.get_drift_corr(s)
                p['yd'] = min(4.0, max(-4.0, 4.0 * (s.yaw_err + dc)))
            else:
                import math as _m
                if abs(s.yaw_err) > L.turn_first:
                    p['bs'] = max(0.0, -3.0 * s.vel)
                    yd = L.turn_gain * s.yaw_err - L.turn_damp * s.yaw_d
                    p['yd'] = max(-L.turn_thrust, min(L.turn_thrust, yd))
                else:
                    sc = max(0.3, min(1.0, dist / L.r_app))
                    t = -L.approach_thrust * sc
                    p['bs'] = t
                    yd = 4.0 * s.yaw_err
                    ylim = L.yd_frac * abs(t)
                    p['yd'] = max(-ylim, min(ylim, yd))

        elif self.state == 'APPROACH':
            ye = s.yaw_err
            if self._aligned:
                if abs(ye) > L.align_tol_out:
                    self._aligned = False
            else:
                if abs(ye) < L.align_tol:
                    self._aligned = True

            if not self._aligned:
                p['bs'] = 0.0
                yd = L.turn_gain * ye - L.turn_damp * s.yaw_d
                p['yd'] = max(-L.turn_thrust, min(L.turn_thrust, yd))
            else:
                v_fwd = -s.vel
                decel = 0.6
                brake_dist = (v_fwd * v_fwd) / (2.0 * decel) if v_fwd > 0 else 0.0
                if s.dist_2d <= brake_dist:
                    p['bs'] = +min(L.approach_thrust, 0.5 * L.cruise_min)
                else:
                    sc = max(0.35, min(1.0, s.dist_2d / L.r_app))
                    p['bs'] = -L.approach_thrust * sc
                yd = 3.0 * ye - 0.5 * s.yaw_d
                ylim = L.yd_frac * L.approach_thrust
                p['yd'] = max(-ylim, min(ylim, yd))

        return p
