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
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    baro_z: float = 0.0
    dz_dt: float = 0.0
    dist_2d: float = 1000.0
    dist_3d: float = 1000.0
    bearing: float = 0.0
    z_err: float = 0.0
    yaw_err: float = 0.0
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
    'Z_STAB': 'Корр.высоты', 'XY_FINAL': 'Сближение',
    'HOVER_STAB': 'Стабилизация', 'FINISH': 'Готово',
}


class Phys:
    P_Z0 = 101325.0
    RHO_G = 9810.0
    DT = 0.05
    MASS = 124.18  # total mass (body + ballasts)
    MAX_BALLAST_VOL = 0.003


class PID:
    # Heading (vertical rudder)
    Kp_yaw = 5.0
    Kd_yaw = 2.8
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
    bz_neutral = 0.267


class Lim:
    rud_spd = 2.4
    cruise = -15.0
    suc_r = 0.5    # success radius
    alt_br = 1.7   # altitude breach threshold


class SR:
    NAV = 6.0
    ZS = 5.0
    XY = 4.0
    BO = 18.0
    HVR = 0.0
