"""Obstacle avoidance — Potential Field method using forward sonar (16×16).

Разбиваем веер сонара 16×16 на сетку секторов 5×3:
    Горизонталь: ЛЕВО | ЦЕНТР-ЛЕВО | ЦЕНТР | ЦЕНТР-ПРАВО | ПРАВО
    Вертикаль:   НИЗ  | ЦЕНТР      | ВЕРХ

Для каждого сектора берём минимальную дистанцию → вычисляем силу
отталкивания. Результат: поправка к курсу (yaw_err) и глубине (z_err),
плюс коэффициент скорости для плавного торможения.

Запас безопасности: 2 м от любого препятствия.
"""
import math


class AvoidanceResult:
    """Результат вычисления обхода препятствий."""
    __slots__ = ('yaw_offset', 'z_offset', 'speed_factor',
                 'emergency', 'obstacle_near')

    def __init__(self):
        self.yaw_offset = 0.0       # рад — добавляется к yaw_err
        self.z_offset = 0.0         # м — добавляется к z_err
        self.speed_factor = 1.0     # 0..1 — множитель тяги
        self.emergency = False       # экстренное торможение
        self.obstacle_near = False   # есть препятствие в зоне влияния


class ObstacleAvoidance:
    """Potential Field: обход препятствий по данным сонара.

    Секторы и направления отталкивания:
        Горизонталь: ЛЕВО → рули вправо, ПРАВО → рули влево
        Вертикаль:   ВЕРХ → погружение, НИЗ → всплытие
    """

    # ── Геометрия сонара (из SDF) ──
    H_SAMPLES = 16
    V_SAMPLES = 16

    # ── Параметры обхода ──
    SAFETY_MARGIN = 2.0         # м — минимальный зазор до препятствия
    INFLUENCE_RANGE = 10.0      # м — начинаем реагировать
    EMERGENCY_RANGE = 3.0       # м — экстренное торможение

    # ── Ограничения выхода ──
    MAX_YAW_OFFSET = 0.7        # рад (~40°) макс. поправка курса
    MAX_Z_OFFSET = 1.5          # м макс. поправка глубины
    YAW_SLEW = 0.5              # рад/с — плавность поправки курса
    Z_SLEW = 0.4                # м/с — плавность поправки глубины

    # ── Поведение при застревании ──
    STUCK_TIMEOUT = 10.0        # с — если не приближаемся, начинаем уход
    MEMORY_DURATION = 5.0       # с — помним последнюю сторону обхода

    # ── Поиск проходов (gap detection) ──
    MIN_GAP_BINS = 2            # минимум подряд «открытых» бинов = проход
    MIN_GAP_WIDTH_M = 0.6       # мин. физическая ширина щели (корпус Ø0.34 м)
    BIN_ANGULAR_RAD = (40.0 / 16.0) * math.pi / 180.0   # ≈0.0436 рад на бин

    def __init__(self):
        self._yaw = 0.0
        self._z = 0.0
        self._memory_side = 0.0     # +1 = вправо, -1 = влево
        self._memory_timer = 0.0
        self._stuck_timer = 0.0
        self._prev_dist = 1000.0

    def reset(self):
        """Сброс состояния (при смене фазы / точки)."""
        self._yaw = 0.0
        self._z = 0.0
        self._memory_timer = 0.0
        self._stuck_timer = 0.0

    def compute(self, sonar_ranges, dist_to_target: float, dt: float):
        """Вычислить поправки для обхода.

        Args:
            sonar_ranges: сырые данные сонара (256 значений).
                          inf / 0 = нет эха.
            dist_to_target: горизонтальная дистанция до точки, м.
            dt: шаг времени, с.

        Returns:
            AvoidanceResult
        """
        result = AvoidanceResult()

        if not sonar_ranges or len(sonar_ranges) < 16:
            return result

        # ── 1. Строим сетку секторов ──
        sectors = self._build_sectors(sonar_ranges)
        # sectors[h][v] = мин. дистанция, inf = чисто

        # ── 2. Ближайшее препятствие ──
        closest = float('inf')
        for row in sectors:
            for d in row:
                if d < closest:
                    closest = d

        # ── Нет угроз — плавно гасим поправки ──
        if closest >= self.INFLUENCE_RANGE:
            self._fade(dt)
            result.yaw_offset = self._yaw
            result.z_offset = self._z
            return result

        # ── 4. Поиск прохода (рано — нужно до emergency) ──
        h_hist = self._build_h_histogram(sonar_ranges)
        gap_yaw = self._gap_steering(h_hist)

        # ── 5. Emergency: стоп только если прохода НЕТ ──
        result.obstacle_near = True
        result.emergency = closest < self.EMERGENCY_RANGE and abs(gap_yaw) < 0.02

        # ── 6. Коэффициент скорости ──
        if closest < self.INFLUENCE_RANGE:
            ratio = (closest - self.SAFETY_MARGIN) / \
                    (self.INFLUENCE_RANGE - self.SAFETY_MARGIN)
            if abs(gap_yaw) > 0.02:
                # Проход есть — не стоп, плавно замедляемся (минимум 30% тяги)
                result.speed_factor = max(0.3, min(1.0, ratio))
            elif result.emergency:
                result.speed_factor = 0.0
            else:
                result.speed_factor = max(0.2, min(1.0, ratio))

        # ── 7. Отталкивание по курсу ──
        yaw_raw = self._yaw_repulsion(sectors)

        # ── 8. Отталкивание по глубине ──
        z_raw = self._z_repulsion(sectors)

        # ── 9. Gap steering: бленд с potential field (gap_yaw уже посчитан) ──
        if abs(gap_yaw) > 0.02:
            # Бленд: чем слабее поле (симметричные препятствия), тем больше
            # доверяем проходу. Сильное поле → 30% gap; слабое → 90% gap.
            conflict = max(0.0, 1.0 - abs(yaw_raw) / 0.3)
            w = 0.3 + 0.6 * conflict
            yaw_raw = yaw_raw * (1.0 - w) + gap_yaw * w

        # ── 10. Память: предпочитаем ту же сторону обхода ──
        if abs(yaw_raw) > 0.05:
            self._memory_side = 1.0 if yaw_raw > 0 else -1.0
            self._memory_timer = self.MEMORY_DURATION
        elif self._memory_timer > 0:
            self._memory_timer -= dt
            # Слабый толчок в ту же сторону, пока память жива
            yaw_raw += self._memory_side * 0.3

        # ── 11. Детектор застревания ──
        approach = self._prev_dist - dist_to_target
        self._prev_dist = dist_to_target

        if approach < 0.05 and result.obstacle_near:
            self._stuck_timer += dt
        else:
            self._stuck_timer = max(0.0, self._stuck_timer - dt * 0.5)

        if self._stuck_timer > self.STUCK_TIMEOUT:
            # Усиленный боковой уход
            yaw_raw += self._memory_side * 0.5
            if self._stuck_timer > self.STUCK_TIMEOUT * 2:
                # Попробуем другую сторону
                self._memory_side *= -1
                self._stuck_timer = 0.0

        # ── 12. Slew-limited выход ──
        yaw_tgt = max(-self.MAX_YAW_OFFSET,
                      min(self.MAX_YAW_OFFSET, yaw_raw))
        z_tgt = max(-self.MAX_Z_OFFSET,
                    min(self.MAX_Z_OFFSET, z_raw))

        dy = self.YAW_SLEW * dt
        dz = self.Z_SLEW * dt
        self._yaw += max(-dy, min(dy, yaw_tgt - self._yaw))
        self._z += max(-dz, min(dz, z_tgt - self._z))

        result.yaw_offset = self._yaw
        result.z_offset = self._z
        return result

    # ═══════════════════════════════════════════
    #  Внутренние методы
    # ═══════════════════════════════════════════

    def _build_sectors(self, ranges):
        """Преобразовать плоский массив ranges в сетку 5×3 мин. дистанций.

        Сетка: sectors[h_sector][v_sector]
            h: 0=ЛЕВО, 1=Ц-ЛЕВО, 2=ЦЕНТР, 3=Ц-ПРАВО, 4=ПРАВО
            v: 0=НИЗ,  1=ЦЕНТР,  2=ВЕРХ

        Раскладка сонара (gz-sim gpu_lidar):
            ranges[v * H_SAMPLES + h]
            h: 0=крайний левый (-20°) → 15=крайний правый (+20°)
            v: 0=нижний (-20°) → 15=верхний (+20°)
        """
        sectors = [[float('inf')] * 3 for _ in range(5)]

        h_per = self.H_SAMPLES // 5   # 3 луча на сектор
        v_per = self.V_SAMPLES // 3   # 5 лучей на сектор

        for hs in range(5):
            h0 = hs * h_per
            h1 = h0 + h_per if hs < 4 else self.H_SAMPLES
            for vs in range(3):
                v0 = vs * v_per
                v1 = v0 + v_per if vs < 2 else self.V_SAMPLES
                d_min = float('inf')
                for v in range(v0, v1):
                    for h in range(h0, h1):
                        idx = v * self.H_SAMPLES + h
                        if idx < len(ranges):
                            r = ranges[idx]
                            if math.isfinite(r) and 0 < r < d_min:
                                d_min = r
                sectors[hs][vs] = d_min

        return sectors

    def _yaw_repulsion(self, sectors):
        """Поправка курса: горизонтальное отталкивание.

        Знаки (yaw_err = bearing - heading):
            +offset → саб поворачивает налево (убегает от правых препятствий)
            -offset → саб поворачивает направо (убегает от левых препятствий)
        """
        # Направления: ЛЕВО → уходим вправо (neg), ПРАВО → уходим влево (pos)
        h_dirs = [-1.0, -0.5, 0.0, +0.5, +1.0]
        # Веса: центральный сектор важнее (прямо по курсу)
        h_weights = [0.6, 0.85, 1.2, 0.85, 0.6]

        repulse = 0.0
        for hs in range(5):
            d = min(sectors[hs])   # худшее по вертикали
            if d < self.INFLUENCE_RANGE:
                prox = (self.INFLUENCE_RANGE - d) / self.INFLUENCE_RANGE
                force = prox * prox     # квадратично: сильно вблизи
                if d < self.SAFETY_MARGIN:
                    force *= 2.5        # паника
                repulse += h_dirs[hs] * force * h_weights[hs]

        return repulse

    def _z_repulsion(self, sectors):
        """Поправка глубины: вертикальное отталкивание.

        Знаки (z_err = pos.z - target.z):
            +offset → саб «выше цели» → контроллер ныряет
            -offset → саб «ниже цели» → контроллер всплывает
        """
        # Направления: ВЕРХ сектора → нырнуть (pos), НИЗ → всплыть (neg)
        v_dirs = [-1.0, 0.0, +1.0]     # НИЗ, ЦЕНТР, ВЕРХ
        v_weights = [0.8, 0.5, 0.8]

        repulse = 0.0
        for vs in range(3):
            d = min(sectors[hs][vs] for hs in range(5))  # худшее по горизонтали
            if d < self.INFLUENCE_RANGE:
                prox = (self.INFLUENCE_RANGE - d) / self.INFLUENCE_RANGE
                force = prox * prox
                if d < self.SAFETY_MARGIN:
                    force *= 2.5
                repulse += v_dirs[vs] * force * v_weights[vs]

        return repulse

    def _fade(self, dt):
        """Плавное затухание поправок, когда препятствий нет."""
        decay = max(0.0, 1.0 - 0.8 * dt)
        self._yaw *= decay
        self._z *= decay

    # ═══════════════════════════════════════════
    #  Поиск проходов (gap detection)
    # ═══════════════════════════════════════════

    def _build_h_histogram(self, ranges):
        """16-бинная горизонтальная гистограмма: мин. дистанция по каждому
        горизонтальному лучу (среди всех 16 вертикальных).

        hist[h] = минимальная дистанция в столбце h.
        inf = в этом направлении ничего нет (чисто).
        """
        hist = [float('inf')] * self.H_SAMPLES
        for h in range(self.H_SAMPLES):
            d_min = float('inf')
            for v in range(self.V_SAMPLES):
                idx = v * self.H_SAMPLES + h
                if idx < len(ranges):
                    r = ranges[idx]
                    if math.isfinite(r) and 0 < r < d_min:
                        d_min = r
            hist[h] = d_min
        return hist

    def _find_passages(self, hist):
        """Найти проходимые щели в горизонтальной гистограмме.

        Используется адаптивный порог: бин считается «открытым», если
        дистанция > 2× nearest_wall (ближайший объект) ИЛИ > nearest + 3м.
        Это работает на любой дальности — от 3м до 50м.

        Дополнительно проверяем физическую ширину щели по дистанции
        до ближайшей стены с каждой стороны.

        Returns:
            [(center_bin, width_bins), ...] — список проходов.
        """
        # Адаптивный порог
        finite = [d for d in hist if math.isfinite(d) and d > 0]
        if not finite:
            return []
        nearest = min(finite)
        if nearest == float('inf'):
            return []  # всё чисто — нет стен
        threshold = max(nearest * 2.0, nearest + 3.0)

        passages = []
        in_gap = False
        start = 0

        for i in range(len(hist)):
            d = hist[i]
            is_open = (not math.isfinite(d)) or d > threshold
            if is_open:
                if not in_gap:
                    start = i
                    in_gap = True
            else:
                if in_gap:
                    self._check_and_add_gap(
                        passages, hist, start, i, nearest)
                    in_gap = False

        # Щель до края гистограммы
        if in_gap:
            self._check_and_add_gap(
                passages, hist, start, len(hist), nearest)

        return passages

    def _check_and_add_gap(self, passages, hist, start, end, nearest):
        """Проверить щель [start, end) на проходимость и добавить в список."""
        width = end - start
        if width < self.MIN_GAP_BINS:
            return
        center = (start + end - 1) / 2.0

        # Физическая ширина щели на расстоянии до ближайшей стены
        left_wall = hist[start - 1] if start > 0 else nearest
        right_wall = hist[end] if end < len(hist) else nearest
        def _fd(x):
            return x if math.isfinite(x) else nearest
        wall_d = min(_fd(left_wall), _fd(right_wall))
        phys_width = wall_d * width * self.BIN_ANGULAR_RAD

        if phys_width >= self.MIN_GAP_WIDTH_M:
            passages.append((center, width))

    def _gap_steering(self, hist):
        """Поправка курса к центру ближайшего прохода.

        Returns:
            yaw offset в рад. 0 если проходов нет.
            Знак: + = поворот налево, − = направо.
        """
        passages = self._find_passages(hist)
        if not passages:
            return 0.0

        # Ближайший к курсу (бин 7.5 = прямо по носу)
        best_center, best_width = min(
            passages, key=lambda p: abs(p[0] - 7.5))

        # Бин → угол → поправка (левые бины → positive yaw → левый поворот)
        angle = (best_center - 7.5) * self.BIN_ANGULAR_RAD
        yaw = -angle

        # Масштаб по ширине (шире проход → увереннее рулим)
        confidence = min(1.0, best_width / 6.0)
        return yaw * confidence
