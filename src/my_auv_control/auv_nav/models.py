"""AUV Data Models (v50.5) — with DepthMode."""
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Dict

class MotorMode(Enum):
    DUAL = auto()
    SINGLE = auto()

class DepthMode(Enum):
    RUDDER  = auto()
    BALLAST = auto()
    BOTH    = auto()

@dataclass
class VehicleState:
    pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    vel: float = 0.0; vel_z: float = 0.0
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    baro_z: float = 0.0; dz_dt: float = 0.0
    dist_2d: float = 1000.0; dist_3d: float = 1000.0
    bearing: float = 0.0; z_err: float = 0.0; yaw_err: float = 0.0
    roll_abs: float = 0.0; pitch_curr: float = 0.0
    pitch_d: float = 0.0; yaw_d: float = 0.0; roll_d: float = 0.0

@dataclass
class ActuatorCommands:
    lt: float = 0.0; rt: float = 0.0
    rv: float = 0.0; hl: float = 0.0; hr: float = 0.0
    ballast_volume: float = 0.5  # 0..1 normalized (0=empty, 1=full)

PHASE_TR = {
    'NAV': 'Круиз', 'BRAKE': 'Торможение',
    'Z_STAB': 'Корр.высоты', 'XY_FINAL': 'Сближение',
    'HOVER_STAB': 'Стабилизация', 'FINISH': 'Готово',
}

class Phys:
    P_Z0 = 101325.0; RHO_G = 9810.0; DT = 0.05; MASS = 118.64
    MAX_BALLAST_VOL = 0.003  # m³ per ballast
    MIN_BALLAST_VOL = 0.0007  # neutral buoyancy
    BALLAST_NEUTRAL = MIN_BALLAST_VOL / MAX_BALLAST_VOL  # 0.233

class PID:
    Kp_z = 10.0; Ki_z = 0.5; Kd_z = 20.0; z_ilim = 3.0
    Kp_yaw = 5.0; Kd_yaw = 2.8
    Kp_roll = 50.0; Kd_roll = 22.0; roll_bias = 0.04
    # Ballast PID
    Kp_bz = 0.8; Ki_bz = 0.02; Kd_bz = 1.5; bz_ilim = 0.3

class Lim:
    rud_spd = 2.4; cruise = -40.0; suc_r = 0.85; alt_br = 1.7
    zs_z = 0.45; zs_dz = 0.06; zs_pred = 0.55
    brk_f = 1.8; brk_v = 0.25; brk_d = 2.0

class SR:
    NAV = 6.0; ZS = 5.0; XY = 4.0; BO = 8.0; HVR = 0.0
