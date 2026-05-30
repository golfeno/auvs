"""LOS-наведение по ПРЯМОЙ между путевыми точками (v83.0).

ПРОБЛЕМА (то, что лечим): чистое наведение «на точку» (pure pursuit) даёт
ДУГИ. Аппарат, снесённый с линии маршрута (инерция, остаточный занос после
разворота, снос течением), возвращается НЕ на саму линию, а к точке — по
кривой погони. Получается дуга там, где можно идти прямо.

РЕШЕНИЕ: классический морской закон LOS (Line-Of-Sight). Аппарат удерживается
на ОТРЕЗКЕ A->B (A — предыдущая точка / старт, B — текущая цель) за счёт
компенсации поперечного сноса (cross-track):

    α      — курс линии  = atan2(By-Ay, Bx-Ax)
    along  =  dx*cosα + dy*sinα   — продвижение вдоль линии
    cross  = -dx*sinα + dy*cosα   — поперечный снос (знак: + = слева от курса)
    χ_d    = α + atan2(-cross, Δ) — желаемый курс (Δ — дистанция упреждения)

cross>0 (аппарат слева от линии) -> atan2(-cross,Δ)<0 -> доворот ВПРАВО ->
возврат на линию. По мере cross->0 χ_d плавно сходится к α -> идём вдоль
линии. Итог: прямой выход на линию + прямой ход по ней вместо дуги.

Работает одинаково для любых координат (плюс/минус) — всё через atan2; нет
никаких допущений о поверхности или знаке Z (Z ведётся отдельным контуром).

OOP: отдельный объект-наводчик; автопилот задаёт ему текущий отрезок
(set_segment) и на каждом шаге вызывает apply(state) — тот переписывает
state.yaw_err по закону LOS. Если отрезок вырожден (A≈B) — наведение
плавно деградирует к «на точку» (bearing).
"""
import math
from .models import Lim as L


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class LOSGuidance:
    """Наведение по линии маршрута (Line-Of-Sight)."""

    def __init__(self):
        self.A = [0.0, 0.0, 0.0]
        self.B = [0.0, 0.0, 0.0]
        self.alpha = 0.0
        self.seg_len = 0.0
        self.valid = False

    def set_segment(self, a, b):
        """Задать активный отрезок A->B (XY-плоскость)."""
        self.A = list(a)
        self.B = list(b)
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        self.seg_len = math.hypot(dx, dy)
        if self.seg_len < L.los_min_seg:
            # Отрезок вырожден (точки почти совпали) — LOS смысла не имеет.
            self.valid = False
            return
        self.alpha = math.atan2(dy, dx)
        self.valid = True

    def apply(self, s) -> None:
        """Переписать s.yaw_err по закону LOS (удержание прямой A->B).

        Метод lookahead-ТОЧКИ (надёжнее формулы бесконечной линии):
          1) проекция аппарата на отрезок, ОГРАНИЧЕННАЯ концами [0, seg_len];
          2) lookahead-точка LP = точка на отрезке в Δ метрах ВПЕРЁД от
             проекции, но НЕ ДАЛЬШЕ B;
          3) желаемый курс = пеленг на LP.

        Это автоматически:
          • держит прямую, когда аппарат на линии (LP впереди по линии);
          • стягивает на линию при сносе (пеленг на точку ВПЕРЁД тянет назад);
          • у конца отрезка и ЗА ним LP == B -> наведение строго НА ТОЧКУ,
            аппарат разворачивается к B (а не уплывает по линии в бесконечность,
            как было с формулой бесконечной прямой).

        Заполняет s.cross_track / s.along_track / s.los_heading для телеметрии.
        При вырожденном отрезке оставляет наведение «на точку».
        """
        if not self.valid:
            s.cross_track = 0.0
            s.along_track = 0.0
            s.los_heading = s.bearing
            # yaw_err уже посчитан в SensorFusion как (bearing - heading)
            return

        ca = math.cos(self.alpha)
        sa = math.sin(self.alpha)
        dx = s.pos[0] - self.A[0]
        dy = s.pos[1] - self.A[1]

        along_raw = dx * ca + dy * sa       # продвижение вдоль линии (может быть >seg_len или <0)
        cross = -dx * sa + dy * ca          # поперечный снос (+ слева)

        # Проекция на отрезок, зажатая концами.
        along = max(0.0, min(self.seg_len, along_raw))

        # lookahead-точка: вперёд на Δ от проекции, но не дальше конца B.
        la = min(along + L.los_lookahead, self.seg_len)
        lpx = self.A[0] + la * ca
        lpy = self.A[1] + la * sa

        # Желаемый курс = пеленг на lookahead-точку.
        # Если аппарат уже у/за B (la==seg_len и LP==B) -> курс прямо на B.
        chi_d = math.atan2(lpy - s.pos[1], lpx - s.pos[0])

        s.cross_track = cross
        s.along_track = along_raw
        s.los_heading = chi_d
        s.yaw_err = _wrap(chi_d - s.rpy[2])

