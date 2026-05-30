"""Data models & configuration (v51.0)."""
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Dict


class MotorMode(Enum):
    DUAL = auto()
    SINGLE = auto()  # Both engines synchronous


class DepthMode(Enum):
    RUDDER = auto()
    BALLAST = auto()
    BOTH = auto()


@dataclass
class VehicleState:
    pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    vel: float = 0.0
    vel_z: float = 0.0
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])       # СЛИТАЯ оценка (используется регуляторами)
    rpy_imu: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # ориентация только по IMU
    rpy_odo: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # ориентация только по одометрии
    gyro: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])      # угловые скорости IMU (p,q,r), рад/с
    imu_ok: bool = False                                                     # приходят ли данные IMU
    mag_heading: float = 0.0   # курс по магнитометру (рад)
    mag_ok: bool = False
    alt_floor: float = -1.0    # высота над дном, м (-1 = нет данных)
    sonar_fwd: float = -1.0    # дистанция до препятствия впереди, м (-1 = чисто)
    sonar_ok: bool = False
    sonar_ranges: list = field(default_factory=list)  # сырые данные сонара (16×16 = 256 значений)
    baro_z: float = 0.0
    dz_dt: float = 0.0
    dist_2d: float = 1000.0
    dist_3d: float = 1000.0
    bearing: float = 0.0
    z_err: float = 0.0
    yaw_err: float = 0.0
    yaw_err_raw: float = 0.0
    # --- LOS-наведение (диагностика прямого хода) ---
    cross_track: float = 0.0   # поперечный снос от линии маршрута, м (+ слева от курса)
    along_track: float = 0.0   # пройдено вдоль линии маршрута, м
    los_heading: float = 0.0   # желаемый курс LOS (рад)
    avoid_mode: str = 'NORMAL' # режим обхода: NORMAL | WALL_FOLLOW
    roll_abs: float = 0.0
    pitch_curr: float = 0.0
    pitch_d: float = 0.0
    yaw_d: float = 0.0
    roll_d: float = 0.0


@dataclass
class ActuatorCommands:
    lt: float = 0.0
    rt: float = 0.0
    rv: float = 0.0
    rvt: float = 0.0  # верхний вертикальный руль — стабилизация крена
    hl: float = 0.0   # кормовой левый горизонтальный руль
    hr: float = 0.0   # кормовой правый горизонтальный руль
    hfl: float = 0.0  # носовой левый горизонтальный руль
    hfr: float = 0.0  # носовой правый горизонтальный руль
    ballast_volume: float = 0.5  # 0..1 normalized


PHASE_TR = {
    'NAV': 'Круиз', 'BRAKE': 'Торможение',
    'Z_STAB': 'Корр.высоты', 'Z_CORRIDOR': 'Коридор-Z', 'APPROACH': 'Сближение',
    'HOVER_STAB': 'Стабилизация', 'FINISH': 'Готово',
}


class Phys:
    P_Z0 = 101325.0
    RHO_G = 9810.0
    DT = 0.02   # 50 Гц — совпадает с частотой одометрии (>=30 Гц)
    MASS = 139.28  # total mass (body 122.7 + ballasts 16.58)
    MAX_BALLAST_VOL = 0.015
    RUDDER_TRIM_VOL = 0.493   # режим рулей: держим баки на нейтрали (как bz_neutral)
    BALLAST_TRIM = 0.05   # статич. дифферент (нос=+, корма=-) из калибровки ballast_neutral


class PID:
    # Heading (vertical rudder) — активный руль в круизе
    Kp_yaw = 9.0     # было 5.0: резче реагирует на курсовую ошибку
    Kd_yaw = 3.5     # было 2.8: чуть больше демпфирования под возросший P
    rud_max = 0.78   # макс. отклонение руля курса (рад, ~45° = предел шарнира SDF)
    # Depth (horizontal rudders)
    Kp_z = 10.0
    Ki_z = 1.5
    Kd_z = 20.0
    z_ilim = 4.0
    # Roll — горизонтальные рули (дифференциально)
    Kp_roll = 50.0
    Kd_roll = 22.0
    roll_bias = 0.04
    # Roll — верхний вертикальный руль (отдельный канал, НЕ инвертируется)
    Kp_roll_v = 8.0
    Kd_roll_v = 3.5
    roll_v_lim = 0.6   # макс. отклонение верхнего верт. руля по крену
    # Носовые горизонтальные рули
    frud_depth_sign = -1.0  # знак канала ГЛУБИНЫ (зеркально кормовым → момент тангажа)
    frud_roll_sign  = 1.0   # знак канала КРЕНА (одинаково с кормовыми → момент крена)
    # Ballast
    Kp_bz = 5.0
    Ki_bz = 0.05
    Kd_bz = 10.0
    bz_ilim = 1.0
    bz_neutral = 0.493   # измерено узлом ballast_neutral (интерполяция корня Vz=0)
    bz_authority = 0.07  # макс. отклонение объёма от нейтрали (|a|<=~0.3 м/с², не 'взлетает')
    bz_slew = 0.033      # макс. скорость изм. norm/с (= 0.001 м3/с / 0.015 м3 насоса)
    bz_vz_max = 0.10     # макс. целевая верт. скорость, м/с (меньше -> меньше раскачки)
    bz_kp_v = 0.35       # усиление внутр. контура по Vz (было 0.6 -> осцилляция)
    bz_kp_z = 0.10       # внеш. контур: z_err -> желаемая Vz (мягче подход к цели)
    bz_ki_v = 0.04       # интеграл внутр. контура (вдвое меньше -> не накачивает качку)
    bz_deadband = 0.12   # м: |z_err|<этого -> держим объём (не дёргаем насос у цели)


class Lim:
    rud_spd = 2.4
    cruise = -15.0
    suc_r = 0.5    # success radius (XY)
    alt_br = 1.7   # altitude breach threshold
    z_corr = 0.5   # коридор по Z: держим |z_err| <= 0.5 м пока идём по XY
    z_corr_in = 0.4  # вход в коридор (с запасом, чтобы не дёргалось у границы)
    cruise_min = 6.0  # мин. ход вперёд пока XY не достигнут (поток для рулей)
    yd_frac = 0.9     # доля диф. курса от тяги (жёстче держит курс, меньше дуги)
    # --- Фаза 3 (Сближение): стоп → разворот на месте → газ по прямой ---
    r_app = 3.0       # радиус сближения: ближе этого включается фаза 3 (или 2b если Z не готов)
    align_tol = 0.1745     # ±10° — навёлся, можно давать газ по прямой
    align_tol_out = 0.349  # ±20° — гистерезис: снова разворот, только если ушли за это
    turn_thrust = 25.0     # ПОЛНЫЙ дифференциал движков для быстрого разворота на месте
    turn_gain = 20.0       # P по yaw_err (быстро доворачивает)
    turn_damp = 60.0       # D по скорости рыскания (гасит перерегулирование/осцилляцию)
    approach_thrust = 7.0  # ход «по прямой» в фазе сближения (умеренный)
    turn_first = 0.35   # рад (~20°): раньше разворачивается на месте -> меньше дуги
    # --- LOS-наведение по прямой (метод lookahead-точки) ---
    los_lookahead = 6.0      # Δ упреждения, м: МЕНЬШЕ -> активнее тянет на линию (резче руль)
    los_lookahead_min = 2.5  # (резерв) нижний предел Δ
    los_max_corr = 0.9       # (резерв) макс. доворот на линию
    los_min_seg = 0.6        # м: короче -> отрезок вырожден, LOS off (наведение на точку)


class SR:
    NAV = 6.0
    ZS = 5.0
    XY = 4.0
    BO = 18.0
    HVR = 0.0
