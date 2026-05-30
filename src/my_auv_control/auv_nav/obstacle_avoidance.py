"""Обход препятствий по сонару 16x16 (v102)."""
import math


class AvoidanceResult:
    __slots__ = ('yaw_offset', 'z_offset', 'speed_factor',
                 'closest', 'emergency', 'obstacle_near', 'mode', '_dir')

    def __init__(self):
        self.yaw_offset = 0.0
        self.z_offset = 0.0
        self.speed_factor = 1.0
        self.emergency = False
        self.obstacle_near = False
        self.mode = 'NORMAL'
        self.closest = float('inf')
        self._dir = None


class ObstacleAvoidance:
    """Gap-follower: всегда вправо при равенстве."""

    H_SAMPLES = 16
    V_SAMPLES = 16
    AVOID_RANGE = 5.0       # м — поворот на 5м от объекта
    MAX_YAW = 0.85
    MAX_Z = 1.8
    MIN_SPEED = 0.45
    HYST = 1.5
    COMMIT_TIME = 1.5

    def __init__(self):
        self._dir = None
        self._committed = 0.0
        self._yaw = 0.0
        self._z = 0.0

    def reset(self):
        self._dir = None
        self._committed = 0.0
        self._yaw = 0.0
        self._z = 0.0

    def compute(self, sonar_ranges, dist_to_target: float, dt: float):
        r = AvoidanceResult()

        if not sonar_ranges or len(sonar_ranges) < self.H_SAMPLES:
            return r

        H, V = self.H_SAMPLES, self.V_SAMPLES
        BIG = 999.0

        def cell(h, v):
            idx = v * H + h
            d = sonar_ranges[idx] if idx < len(sonar_ranges) else float('inf')
            return d if (math.isfinite(d) and d > 0.0) else BIG

        r.closest = min(cell(h, v) for h in range(H) for v in range(V))

        if r.closest > self.AVOID_RANGE:
            self._yaw *= max(0.0, 1.0 - 2.0 * dt)
            self._z *= max(0.0, 1.0 - 2.0 * dt)
            if abs(self._yaw) < 0.02 and abs(self._z) < 0.02:
                self._dir = None
            r.yaw_offset = self._yaw
            r.z_offset = self._z
            r.speed_factor = 1.0
            return r

        r.obstacle_near = True

        left  = min(cell(h, v) for h in (0, 1, 2)   for v in range(V))
        right = min(cell(h, v) for h in (13, 14, 15) for v in range(V))
        down  = min(cell(h, v) for v in (0, 1, 2)   for h in range(H))
        up    = min(cell(h, v) for v in (13, 14, 15) for h in range(H))

        options = {'L': left, 'R': right, 'U': up, 'D': down}
        # Всегда вправо при равенстве
        best = 'R'
        for dir_name, clearance in options.items():
            if clearance > options[best] + 0.1:
                best = dir_name

        if self._dir is None:
            self._dir = best
            self._committed = 0.0
        else:
            self._committed += dt
            current_clear = options[self._dir]
            if (self._committed >= self.COMMIT_TIME and
                    current_clear < self.AVOID_RANGE and
                    options[best] > current_clear + self.HYST):
                self._dir = best
                self._committed = 0.0

        clear = options[self._dir]
        need = max(0.0, (self.AVOID_RANGE - min(clear, self.AVOID_RANGE)) / max(0.1, self.AVOID_RANGE))
        auth = max(0.4, min(1.0, need))

        # Если препятствие ДАЛЕКО (>3м) — мягкий поворот, БЛИЗКО — резкий
        if r.closest > 3.0:
            auth *= max(0.3, r.closest / self.AVOID_RANGE)  # плавно нарастает

        yaw_t, z_t = 0.0, 0.0
        if self._dir == 'L':
            yaw_t = +self.MAX_YAW * auth
        elif self._dir == 'R':
            yaw_t = -self.MAX_YAW * auth
        elif self._dir == 'D':
            z_t = +self.MAX_Z * auth
        elif self._dir == 'U':
            z_t = -self.MAX_Z * auth

        self._yaw = yaw_t
        self._z = z_t

        r.yaw_offset = self._yaw
        r.z_offset = self._z
        r.speed_factor = max(self.MIN_SPEED, min(1.0, r.closest / self.AVOID_RANGE))
        r.mode = 'AVOID-' + self._dir
        r._dir = self._dir
        return r
