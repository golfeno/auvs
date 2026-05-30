"""Обход препятствий — ВЗВЕШЕННЫЙ VFH + антиколебание, v117.

Идея VFH: из 16 гор. лучей строим «стоимость» каждого направления и едем в
сторону минимальной стоимости. В отличие от бинарного (занят/свободен) здесь у
КАЖДОГО луча СВОЯ стоимость по дальности: чем ближе препятствие в секторе, тем
дороже туда рулить (и тем сильнее «дорожает» соседям — мягкая инфляция по радиусу
аппарата). Плюс к цели тянет «целевой» штраф, а к ранее выбранному курсу —
«инерционный» (убирает дрожание).

Стоимость сектора h:
    cost(h) = K_OBS * obstacle(h)            # близость помех (своя + соседей)
            + K_GOAL * |angle(h) - goal|     # отклонение от направления на цель
            + K_PREV * |angle(h) - committed| # инерция (держим прежний курс)
Выбираем h с МИНИМАЛЬНОЙ стоимостью -> абсолютный desired_heading.

obstacle(h) = Σ_j  prox(j) * falloff(|h-j|),
  prox(j) = clamp((DETECT - d_j)/DETECT, 0..1)   # 0 далеко, 1 вплотную
  falloff — треугольное ядро шириной INFLATE (раздувание на радиус аппарата).

АНТИКОЛЕБАНИЕ: если знак выбранного отворота часто меняется (аппарат «мечется»
±сектор туда-сюда), детектор это ловит и ВКЛЮЧАЕТ HOLD: фиксирует текущий курс
на OSC_HOLD_TIME сек (резко поднимает K_PREV и не пересматривает сторону) и
сбрасывает скорость — аппарат проходит мимо по одной дуге, а не дёргается.

Геометрия: сонар 16×16, ±45°. Гор. профиль = min по вертикали (ближайшее в столбце).
SECTOR_SIGN переворачивает лево/право, если оси сенсора инвертированы.
"""
import math


class AvoidanceResult:
    __slots__ = ('yaw_offset', 'z_offset', 'speed_factor',
                 'closest', 'emergency', 'obstacle_near', 'mode', '_dir',
                 'desired_heading', 'has_heading',
                 'zL', 'zR', 'zU', 'zD')

    def __init__(self):
        self.yaw_offset = 0.0
        self.z_offset = 0.0
        self.speed_factor = 1.0
        self.emergency = False
        self.obstacle_near = False
        self.mode = 'NORMAL'
        self.closest = float('inf')
        self._dir = None
        self.desired_heading = 0.0
        self.has_heading = False
        self.zL = 999.0; self.zR = 999.0; self.zU = 999.0; self.zD = 999.0


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ObstacleAvoidance:
    """Взвешенный VFH с антиколебанием."""

    H_SAMPLES = 16
    V_SAMPLES = 16
    FOV = 1.0471975            # полный гор. обзор ±30° (=60°, совпадает с SDF сонара)
    SECTOR_SIGN = -1.0         # знак угла сектора (инвертирован: оси сенсора повёрнуты
                               # из-за body +90° по Y; при +1.0 рулил В объект -> уперся в центр)

    AVOID_RANGE = 12.0         # м — порог входа по умолчанию (маршевые фазы)
    AVOID_RANGE_FAR = 12.0     # м — дальние фазы (круиз NAV, коридор Z_CORRIDOR)
    AVOID_RANGE_NEAR = 5.0     # м — ближние фазы (Z_STAB / APPROACH / HOVER_STAB)
    DETECT = 12.0              # м — шкала «близости»: на этой дист. prox=0, у носа=1
    INFLATE = 3                # ширина ядра раздувания (секторов, ~радиус аппарата)

    # Веса стоимости
    K_OBS = 6.0                # вес близости препятствий
    K_GOAL = 1.0              # вес отклонения от цели (рад)
    K_PREV = 0.6               # вес инерции (держать прежний курс), рад

    PASS_TIME = 3.0            # с: держим курс после ложного «чисто» (узкий конус)
    MIN_SPEED = 0.3            # мин. множитель скорости у помехи: НИКОГДА не 0,
                               # аппарат всегда сохраняет ход (поток на рулях, не виснет)
    BRAKE_RANGE = 2.5          # м — ближе этого РЕЗКО тормозим (реверс тяги в node)

    # Антиколебание
    OSC_WINDOW = 25            # тиков истории (~0.5 с при 50 Гц)
    OSC_FLIPS = 4              # столько смен знака отворота в окне = «мечется»
    OSC_HOLD_TIME = 4.0        # с: на сколько фиксируем курс при детекте колебаний
    K_PREV_HOLD = 8.0          # усиленная инерция в режиме HOLD (жёстко держим курс)

    def __init__(self):
        self._committed_sector = None
        self._committed_heading = 0.0
        self._pass = 0.0
        self._sign_hist = []       # история знака yaw_offset
        self._osc_hold = 0.0       # таймер режима HOLD (антиколебание)

    def reset(self):
        self._committed_sector = None
        self._committed_heading = 0.0
        self._pass = 0.0
        self._sign_hist = []
        self._osc_hold = 0.0

    def _sector_angle(self, h):
        step = self.FOV / (self.H_SAMPLES - 1)
        return self.SECTOR_SIGN * (self.FOV * 0.5 - h * step)

    def compute(self, sonar_ranges, heading: float, bearing: float, dt: float):
        r = AvoidanceResult()
        H, V = self.H_SAMPLES, self.V_SAMPLES

        if not sonar_ranges or len(sonar_ranges) < H:
            return r

        BIG = 999.0

        def cell(h, v):
            idx = v * H + h
            d = sonar_ranges[idx] if idx < len(sonar_ranges) else float('inf')
            return d if (math.isfinite(d) and d > 0.0) else BIG

        # Гор. профиль: ближайшая дистанция в каждом столбце.
        prof = [min(cell(h, v) for v in range(V)) for h in range(H)]
        r.closest = min(prof)
        r.zL = min(prof[:3]); r.zR = min(prof[-3:]); r.zU = r.zD = 999.0

        # ── Нет помехи в зоне обхода ──
        if r.closest > self.AVOID_RANGE:
            if self._committed_sector is not None and self._pass > 0.0:
                self._pass = max(0.0, self._pass - dt)
                self._osc_hold = max(0.0, self._osc_hold - dt)
                r.obstacle_near = True
                r.desired_heading = self._committed_heading
                r.has_heading = True
                r.speed_factor = self.MIN_SPEED
                r.mode = 'PASS'
                r._dir = 'C'
                r.yaw_offset = _wrap(self._committed_heading - heading)
                return r
            self._committed_sector = None
            self._sign_hist.clear()
            self._osc_hold = 0.0
            r.speed_factor = 1.0
            r.mode = 'NORMAL'
            return r

        r.obstacle_near = True
        self._pass = self.PASS_TIME

        # ── 1) Близость препятствий по секторам: prox=0 далеко .. 1 вплотную ──
        prox = [max(0.0, min(1.0, (self.DETECT - prof[h]) / self.DETECT))
                for h in range(H)]

        # ── 2) Раздувание: стоимость препятствий = свёртка с треугольным ядром ──
        #     (близкая помеха «дорожает» себе и соседям на радиус аппарата).
        obstacle = [0.0] * H
        for h in range(H):
            acc = 0.0
            for k in range(-self.INFLATE, self.INFLATE + 1):
                hh = h + k
                if 0 <= hh < H:
                    w = 1.0 - abs(k) / (self.INFLATE + 1.0)   # треуг. ядро
                    acc += prox[hh] * w
            obstacle[h] = acc

        # ── 3) Целевой и инерционный термы ──
        goal_ang = _wrap(bearing - heading)
        prev_ang = (_wrap(self._committed_heading - heading)
                    if self._committed_sector is not None else goal_ang)

        # В режиме HOLD держим курс жёстко (резко поднимаем инерцию).
        k_prev = self.K_PREV_HOLD if self._osc_hold > 0.0 else self.K_PREV

        # ── 4) Полная стоимость каждого сектора, выбираем минимум ──
        best_h, best_cost = 0, float('inf')
        for h in range(H):
            ang = self._sector_angle(h)
            cost = (self.K_OBS * obstacle[h]
                    + self.K_GOAL * abs(ang - goal_ang)
                    + k_prev * abs(ang - prev_ang))
            if cost < best_cost:
                best_cost, best_h = cost, h

        # ── 5) В режиме HOLD сторону НЕ меняем (фиксируем курс), иначе коммитим ──
        if self._osc_hold > 0.0 and self._committed_sector is not None:
            self._osc_hold = max(0.0, self._osc_hold - dt)
            sec = self._committed_sector
        else:
            sec = best_h
            self._committed_sector = sec

        self._committed_heading = _wrap(heading + self._sector_angle(sec))
        yaw_off = _wrap(self._committed_heading - heading)

        # ── 6) Детектор колебаний: считаем смены знака отворота в окне ──
        sg = 0 if abs(yaw_off) < 0.05 else (1 if yaw_off > 0 else -1)
        self._sign_hist.append(sg)
        if len(self._sign_hist) > self.OSC_WINDOW:
            self._sign_hist.pop(0)
        flips = 0
        prev = 0
        for v in self._sign_hist:
            if v != 0 and prev != 0 and v != prev:
                flips += 1
            if v != 0:
                prev = v
        if flips >= self.OSC_FLIPS and self._osc_hold <= 0.0:
            # Аппарат «мечется» -> фиксируем ТЕКУЩИЙ курс на OSC_HOLD_TIME.
            self._osc_hold = self.OSC_HOLD_TIME

        r.desired_heading = self._committed_heading
        r.has_heading = True
        r.yaw_offset = yaw_off
        r.speed_factor = max(self.MIN_SPEED, min(1.0, r.closest / self.AVOID_RANGE))
        ang = self._sector_angle(sec)
        side = 'L' if ang > 0.02 else ('R' if ang < -0.02 else 'C')
        r._dir = side
        r.mode = ('HOLD-' if self._osc_hold > 0.0 else 'AVOID-') + side
        return r
