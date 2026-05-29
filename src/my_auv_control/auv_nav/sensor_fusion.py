"""Sensor Fusion (v52.0) — IMU + Odometry + комплементарный фильтр.

Источники ориентации:
  • ОДОМЕТРИЯ  (50 Гц): абсолютная ориентация из позы (без дрейфа, но «грубее»,
                низкая частота, может запаздывать).
  • IMU        (100 Гц): гироскоп даёт чистые угловые скорости (быстро, малошумно
                на коротком окне), но интегрирование угла даёт ДРЕЙФ.

Слияние — КОМПЛЕМЕНТАРНЫЙ ФИЛЬТР (classic complementary filter):
    angle = a * (angle + gyro * dt) + (1 - a) * angle_odom
  - высокочастотную часть берём из гироскопа (быстрый отклик),
  - низкочастотную (опорную, без дрейфа) — из одометрии.
  Это упрощённый аналог фильтра Калмана для AHRS; широко применяется в навигации,
  когда полноценный EKF избыточен.
"""
import math
from typing import List
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from .models import VehicleState, Phys


def _quat_to_rpy(q):
    r = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
    p = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
    y = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
    return [r, p, y]


def _wrap(a):
    """Нормализация угла в [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class SensorFusion:
    # Коэффициент комплементарного фильтра (доля гироскопа).
    # 0.98 → 98% гироскоп / 2% одометрия. Постоянная времени ~ a*dt/(1-a).
    ALPHA = 0.98

    def __init__(self, node: Node):
        self.node = node
        self.state = VehicleState()
        self._pb = 0.0
        self._pr = [0.0, 0.0, 0.0]
        self._fused = [0.0, 0.0, 0.0]   # текущая слитая ориентация
        self._fused_init = False
        self._last_imu_t = None

        self.node.create_subscription(Odometry, '/model/submarine/odometry', self._odom, 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.node.create_subscription(Float32, '/model/submarine/pressure', self._press, qos)
        self.node.create_subscription(Imu, '/model/submarine/imu', self._imu, qos)

    def _press(self, msg):
        self.state.baro_z = (Phys.P_Z0 - msg.data) / Phys.RHO_G

    def _imu(self, msg):
        """IMU: гироскоп (угл. скорости) + интегрирование в комплем. фильтре."""
        s = self.state
        s.imu_ok = True
        # Угловые скорости (рад/с): p=roll_rate, q=pitch_rate, r=yaw_rate
        s.gyro[0] = msg.angular_velocity.x
        s.gyro[1] = msg.angular_velocity.y
        s.gyro[2] = msg.angular_velocity.z
        # Абсолютная ориентация из IMU (для отдельного вывода)
        s.rpy_imu = _quat_to_rpy(msg.orientation)

        # Время для интегрирования гироскопа
        t = self.node.get_clock().now().nanoseconds / 1e9
        if self._last_imu_t is None:
            self._last_imu_t = t
            return
        dt = t - self._last_imu_t
        self._last_imu_t = t
        if dt <= 0 or dt > 0.5:
            return

        if not self._fused_init:
            # инициализируемся опорой на одометрию (если есть) или IMU
            self._fused = list(s.rpy_odo) if any(s.rpy_odo) else list(s.rpy_imu)
            self._fused_init = True

        a = self.ALPHA
        for i in range(3):
            # high-pass: интеграл гироскопа; low-pass: опорный угол одометрии
            pred = self._fused[i] + s.gyro[i] * dt
            ref = s.rpy_odo[i]
            # корректный учёт перехода через ±pi
            err = _wrap(ref - pred)
            self._fused[i] = _wrap(pred + (1.0 - a) * err)

    def _odom(self, msg):
        s = self.state
        s.pos[0] = msg.pose.pose.position.x
        s.pos[1] = msg.pose.pose.position.y
        s.pos[2] = self.state.baro_z
        s.vel = msg.twist.twist.linear.x
        s.vel_z = msg.twist.twist.linear.z
        # Ориентация только по одометрии
        s.rpy_odo = _quat_to_rpy(msg.pose.pose.orientation)
        if not self._fused_init and not s.imu_ok:
            # без IMU слитая = одометрия
            self._fused = list(s.rpy_odo)

    def update(self, target: List[float], dt: float):
        s = self.state
        # --- выбор итоговой ориентации: слияние, если есть IMU; иначе одометрия ---
        if s.imu_ok and self._fused_init:
            s.rpy = list(self._fused)
        else:
            s.rpy = list(s.rpy_odo)

        dx, dy, dz = target[0]-s.pos[0], target[1]-s.pos[1], target[2]-s.pos[2]
        s.dist_2d = math.hypot(dx, dy)
        s.dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)
        s.bearing = math.atan2(dy, dx)
        s.z_err = s.pos[2] - target[2]
        s.roll_abs = abs(s.rpy[0])
        s.pitch_curr = s.rpy[1]
        raw_dz = (s.pos[2] - self._pb) / dt
        s.dz_dt = 0.6 * s.dz_dt + 0.4 * raw_dz
        self._pb = s.pos[2]
        s.yaw_err = math.atan2(math.sin(s.bearing - s.rpy[2]), math.cos(s.bearing - s.rpy[2]))
        # Угловые скорости: при наличии IMU берём гироскоп (чище), иначе численную производную
        if s.imu_ok:
            s.roll_d = s.gyro[0]
            s.pitch_d = s.gyro[1]
            s.yaw_d = s.gyro[2]
        else:
            s.roll_d = (s.rpy[0] - self._pr[0]) / dt
            s.pitch_d = (s.rpy[1] - self._pr[1]) / dt
            s.yaw_d = (s.rpy[2] - self._pr[2]) / dt
        self._pr = list(s.rpy)
