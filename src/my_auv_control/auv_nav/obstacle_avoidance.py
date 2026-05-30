"""Обход препятствий по переднему сонару 16×16 (v92).

ПРОСТАЯ И ЧЕСТНАЯ ЛОГИКА — 4 РАВНОПРАВНЫХ направления ухода:
    ВЛЕВО, ВПРАВО, ВВЕРХ, ВНИЗ.
Для каждого меряем, насколько там свободно (дистанция по краю веера сонара в
ту сторону). Выбираем НАИБОЛЕЕ свободное. Если выигрывает горизонталь — правим
курс (yaw_offset); если вертикаль — правим глубину (z_offset, рулями ИЛИ баками,
тут без разницы — это просто целевое смещение по Z).

Антидребезг: один раз выбранное направление держим, пока новое не станет
ЗАМЕТНО (на HYST метров) свободнее — тогда переключаемся. Скорость у препятствия
снижаем, но не до нуля (чтобы был ход/поток и сонар смотрел вперёд).

Выход: yaw_offset (+влево/−вправо, добавка к курсовой ошибке),
z_offset (+вниз/−вверх, добавка к ошибке глубины), speed_factor (0..1),
obstacle_near, mode (для телеметрии).
"""
import math


class AvoidanceResult:
    __slots__ = ('yaw_offset', 'z_offset', 'speed_factor',
                 'emergency', 'obstacle_near', 'mode', 'yaw_override')

    def __init__(self):
        self.yaw_offset = 0.0
        self.z_offset = 0.0
        self.speed_factor = 1.0
        self.emergency = False
        self.obstacle_near = False
        self.mode = 'NORMAL'
        self.yaw_override = False   # совместимость; не используется


class ObstacleAvoidance:
    """Выбор самого свободного из 4 направлений ухода (лево/право/вверх/вниз)."""

    # ── Геометрия сонара (SDF: 16×16, ±20°) ──
    H_SAMPLES = 16
    V_SAMPLES = 16

    # ── Дистанции ──
    INFLUENCE_RANGE = 20.0   # м — реагируем СИЛЬНО заранее (большой запас)
    FULL_AUTH_RANGE = 13.0   # м — полный авторитет ухода уже здесь
    SLOW_RANGE = 8.0         # м — ближе этого снижаем скорость
    MIN_SPEED = 0.4          # мин. множитель тяги (не до нуля -> нет спина)
    CLEAR_GOAL = 4.0         # м — целевой боковой зазор (чтобы при пересечении
                             # плоскости сигнала корпус был уже ≥1 м от помехи)

    # ── Авторитет ухода ──
    MAX_YAW = 0.85           # рад (~49°) макс. поправка курса (был 0.7)
    MAX_Z = 1.8             # м макс. поправка глубины
    HYST = 2.0              # м — насколько новое направление должно быть свободнее,
                           # чтобы переключиться (антидребезг выбора)

    COMMIT_TIME = 2.5        # с — после выбора держим направление минимум столько
    PASS_TIME = 4.0          # с — ПОСЛЕ того как фронт расчистился, держим уход
                             # ещё столько (аппарат проходит мимо помехи, а не
                             # срезает угол: препятствие ушло из узкого конуса
                             # сонара вбок, но физически ещё рядом).

    def __init__(self):
        self._dir = None     # текущее выбранное направление: 'L','R','U','D' или None
        self._committed = 0.0  # сколько секунд держим текущее направление
        self._pass = 0.0       # таймер «проход мимо» после расчистки фронта
        self._yaw = 0.0
        self._z = 0.0

    def reset(self):
        self._dir = None
        self._committed = 0.0
        self._pass = 0.0
        self._yaw = 0.0
        self._z = 0.0

    def compute(self, sonar_ranges, dist_to_target: float, dt: float):
        r = AvoidanceResult()
        if not sonar_ranges or len(sonar_ranges) < self.H_SAMPLES:
            self._relax(dt); r.yaw_offset = self._yaw; r.z_offset = self._z
            return r

        H, V = self.H_SAMPLES, self.V_SAMPLES

        # Свободное расстояние по 4 краевым зонам веера + фронт по центру.
        # Берём БЛИЖАЙШИЙ объект в зоне (min), а не среднее: близкая помеха не
        # «размывается» -> разница между свободной и занятой стороной РЕЗКАЯ,
        # выбор и сила ухода решают сильнее.
        BIG = self.INFLUENCE_RANGE * 2.0
        def cell(h, v):
            idx = v * H + h
            d = sonar_ranges[idx] if idx < len(sonar_ranges) else float('inf')
            return d if (math.isfinite(d) and d > 0.0) else BIG

        # центр (фронт) — ШИРОКАЯ центральная зона (4 столбца × 4 ряда), чтобы
        # УГОЛ объекта, попавший в край носового сектора, тоже считался «впереди»
        # (раньше узкий центр 2×2 не видел угол -> аппарат утыкался носом в край).
        front = min(cell(h, v) for h in (6, 7, 8, 9) for v in (6, 7, 8, 9))

        # 4 края: ближайшее препятствие в полосе (min по всей полосе)
        left  = min(cell(h, v) for h in (0, 1, 2)   for v in range(V))
        right = min(cell(h, v) for h in (13, 14, 15) for v in range(V))
        down  = min(cell(h, v) for v in (0, 1, 2)   for h in range(H))
        up    = min(cell(h, v) for v in (13, 14, 15) for h in range(H))

        # ── Чисто впереди ──
        if front >= self.INFLUENCE_RANGE:
            # ПРОХОД МИМО: если только что обходили — НЕ бросаем манёвр сразу.
            # Помеха ушла из узкого конуса сонара вбок, но корпус ещё рядом с ней.
            # Держим текущий уход ещё PASS_TIME секунд (тает к нулю) -> проходим
            # МИМО, а не срезаем угол обратно к цели.
            if self._dir is not None and self._pass > 0.0:
                self._pass -= dt
                # держим набранную поправку курса/глубины (полный ход вперёд) —
                # проходим мимо помехи. По истечении таймера -> обычный relax.
                r.obstacle_near = True
                r.speed_factor = 1.0
                r.yaw_offset = self._yaw
                r.z_offset = self._z
                r.mode = 'PASS-' + self._dir
                return r
            self._relax(dt)
            r.yaw_offset = self._yaw; r.z_offset = self._z
            return r

        r.obstacle_near = True
        self._pass = self.PASS_TIME   # фронт занят -> взводим таймер прохода

        # ── Выбор направления: самое свободное из 4 ──
        # КЛЮЧ против дребезга: выбранное направление МЕНЯЕМ только если оно само
        # стало тесным (clearance < INFLUENCE). Пока в выбранную сторону свободно
        # — держим её, даже если другая чуть свободнее (нос рыскает -> зоны
        # «дышат», но мы не перекладываемся туда-сюда).
        options = {'L': left, 'R': right, 'U': up, 'D': down}
        best = max(options, key=options.get)
        if self._dir is None:
            self._dir = best
            self._committed = 0.0
        else:
            self._committed += dt
            # Менять направление можно только ПОСЛЕ commit-времени И только если
            # выбранная сторона стала тесной и есть заметно лучшая. Это убирает
            # дребезг при рыскании носа (зоны «дышат», но мы держим выбор).
            if (self._committed >= self.COMMIT_TIME and
                    options[self._dir] < self.INFLUENCE_RANGE * 0.7 and
                    options[best] > options[self._dir] + self.HYST):
                self._dir = best
                self._committed = 0.0

        # ── Сила ухода ──
        # (а) близость: полный авторитет достигается уже на FULL_AUTH_RANGE
        #     (раньше, чем вплотную) -> аппарат успевает увести КОРПУС, а не
        #     задеть препятствие краем/рулём.
        prox = (self.INFLUENCE_RANGE - front) / (self.INFLUENCE_RANGE - self.FULL_AUTH_RANGE)
        prox = max(0.0, min(1.0, prox))
        # (б) держим ПОЛНУЮ силу, пока выбранная сторона не станет реально
        #     свободной (clearance >= CLEAR_GOAL). Чем теснее выбранная сторона —
        #     тем сильнее доворачиваем (нужен запас, чтобы корпус прошёл с >1 м).
        clear = options[self._dir]
        need = max(0.0, (self.CLEAR_GOAL - min(clear, self.CLEAR_GOAL)) / self.CLEAR_GOAL)
        gain = max(0.4, need)        # 1.0 пока сторона тесная -> 0.4 когда уже чисто
        auth = max(prox, gain) if prox > 0 else 0.0
        auth = min(1.0, auth)
        yaw_t, z_t = 0.0, 0.0
        if self._dir == 'L':
            yaw_t = +self.MAX_YAW * auth
        elif self._dir == 'R':
            yaw_t = -self.MAX_YAW * auth
        elif self._dir == 'D':
            z_t = +self.MAX_Z * auth   # z_err>0 -> ниже цели -> контроллер вниз
        elif self._dir == 'U':
            z_t = -self.MAX_Z * auth

        # плавный подвод (slew по выходу — без рывков, но быстро)
        self._yaw += max(-3.0 * dt, min(3.0 * dt, yaw_t - self._yaw))
        self._z += max(-2.0 * dt, min(2.0 * dt, z_t - self._z))

        # ── Скорость: ниже у препятствия, но не до нуля ──
        if front >= self.SLOW_RANGE:
            r.speed_factor = 1.0
        else:
            r.speed_factor = max(self.MIN_SPEED, front / self.SLOW_RANGE)

        r.yaw_offset = self._yaw
        r.z_offset = self._z
        r.mode = 'AVOID-' + self._dir
        return r

    def _relax(self, dt):
        """Чисто впереди: плавно убираем поправки, сбрасываем выбор."""
        k = max(0.0, 1.0 - 2.0 * dt)
        self._yaw *= k
        self._z *= k
        if abs(self._yaw) < 0.02 and abs(self._z) < 0.02:
            self._dir = None
            self._committed = 0.0
