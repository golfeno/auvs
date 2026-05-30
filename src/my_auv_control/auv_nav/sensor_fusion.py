"""Sensor Fusion (v53.0) — фильтр Калмана (IMU + Одометрия + Магнитометр) + Сонар/Альтиметр.

Оценка ориентации — линейный фильтр Калмана по каждой оси (roll, pitch, yaw):
  Состояние:  x = угол (рад)
  ПРОГНОЗ (predict): из гироскопа IMU (быстро, 100 Гц), но он копит дрейф →
      x⁻ = x + gyro·dt ;   P⁻ = P + Q
  КОРРЕКЦИЯ (update): по абсолютному измерению (одометрия для roll/pitch/yaw;
      дополнительно магнитометр для yaw — убирает дрейф курса):
      K = P⁻ / (P⁻ + R) ;   x = x⁻ + K·wrap(z − x⁻) ;   P = (1−K)·P⁻
  Q — доверие к модели гироскопа, R — шум измерения. Меньше R → больше веры
  измерению. Это классический алгоритм навигации (упрощённый AHRS-EKF до 1D).

Дополнительные датчики (не входят в оценку ориентации, дают ситуац. данные):
  • МАГНИТОМЕТР → абсолютный курс (коррекция yaw в фильтре).
  • АЛЬТИМЕТР (эхолот вниз) → высота над дном.
  • СОНАР (эхолот вперёд) → дистанция до ближайшего препятствия.
"""
import math
from typing import List
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField, LaserScan
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from .models import VehicleState, Phys


def _quat_to_rpy(q):
    r = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
    p = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
    y = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
    return [r, p, y]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class _Kalman1D:
    """Скалярный фильтр Калмана для одного угла (predict по gyro, update по абс. измерению)."""
    def __init__(self, q=1e-4, r=2e-2):
        self.x = 0.0      # оценка угла
        self.P = 1.0      # ковариация ошибки
        self.Q = q        # шум процесса (доверие к гироскопу)
        self.R = r        # шум измерения (одометрия)
        self.init = False

    def predict(self, gyro, dt):
        if not self.init:
            return
        self.x = _wrap(self.x + gyro * dt)
        self.P += self.Q

    def update(self, z, R=None):
        R = self.R if R is None else R
        if not self.init:
            self.x = z
            self.P = 1.0
            self.init = True
            return
        K = self.P / (self.P + R)
        self.x = _wrap(self.x + K * _wrap(z - self.x))
        self.P = (1.0 - K) * self.P


class SensorFusion:
    def __init__(self, node: Node):
        self.node = node
        self.state = VehicleState()
        self._pb = 0.0
        self._pr = [0.0, 0.0, 0.0]
        self._last_imu_t = None
        # Три независимых фильтра Калмана: roll, pitch, yaw
        self.kf = [_Kalman1D(q=1e-4, r=2e-2),    # roll
                   _Kalman1D(q=1e-4, r=2e-2),    # pitch
                   _Kalman1D(q=1e-4, r=3e-2)]    # yaw (одометрия) + магнитометр

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.node.create_subscription(Odometry, '/model/submarine/odometry', self._odom, 10)
        self.node.create_subscription(Float32, '/model/submarine/pressure', self._press, qos)
        self.node.create_subscription(Imu, '/model/submarine/imu', self._imu, qos)
        self.node.create_subscription(MagneticField, '/model/submarine/magnetometer', self._mag, qos)
        self.node.create_subscription(LaserScan, '/model/submarine/altimeter', self._alt, qos)
        self.node.create_subscription(LaserScan, '/model/submarine/sonar', self._sonar, qos)

    # ---------- сенсоры ----------
    def _press(self, msg):
        self.state.baro_z = (Phys.P_Z0 - msg.data) / Phys.RHO_G

    def _imu(self, msg):
        """IMU: гироскоп → шаг ПРОГНОЗА фильтра Калмана.

        ВАЖНО: звено body создано повёрнутым на +90° вокруг Y (pose ...0 1.5708 0),
        чтобы цилиндр лежал горизонтально. Поэтому оси IMU повёрнуты относительно
        мировой системы (в которой работает одометрия и регуляторы):
            world = Rot_y(90) * body  ⇒  [x,y,z]_world = [z, y, -x]_body
        Ремапим и угловые скорости, и углы IMU в мировую систему.
        """
        s = self.state
        s.imu_ok = True
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        # body → world: roll_rate=gz, pitch_rate=gy, yaw_rate=-gx
        s.gyro[0] = gz
        s.gyro[1] = gy
        s.gyro[2] = -gx
        r_imu = _quat_to_rpy(msg.orientation)
        # body → world для углов (та же перестановка): roll=imu_yaw, pitch=imu_pitch, yaw=-imu_roll
        s.rpy_imu = [r_imu[2], r_imu[1], -r_imu[0]]

        t = self.node.get_clock().now().nanoseconds / 1e9
        if self._last_imu_t is None:
            self._last_imu_t = t
            return
        dt = t - self._last_imu_t
        self._last_imu_t = t
        if dt <= 0 or dt > 0.5:
            return
        for i in range(3):
            self.kf[i].predict(s.gyro[i], dt)

    def _mag(self, msg):
        """Магнитометр → абсолютный курс (yaw), КОРРЕКЦИЯ фильтра."""
        s = self.state
        mx, my = msg.magnetic_field.x, msg.magnetic_field.y
        s.mag_heading = math.atan2(-my, mx)   # курс из горизонтальных компонент
        s.mag_ok = True
        # магнитометру доверяем умеренно (R больше, чем у одометрии)
        self.kf[2].update(s.mag_heading, R=8e-2)

    def _alt(self, msg):
        """Альтиметр (луч вниз) → высота над дном."""
        s = self.state
        rng = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        s.alt_floor = min(rng) if rng else -1.0

    def _sonar(self, msg):
        """Сонар (веер вперёд) → дистанция до ближайшего препятствия."""
        s = self.state
        s.sonar_ranges = list(msg.ranges)    # полные данные для obstacle_avoidance
        rng = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        s.sonar_fwd = min(rng) if rng else -1.0
        s.sonar_ok = True

    def _odom(self, msg):
        s = self.state
        s.pos[0] = msg.pose.pose.position.x
        s.pos[1] = msg.pose.pose.position.y
        s.pos[2] = self.state.baro_z
        s.vel = msg.twist.twist.linear.x
        s.vel_z = msg.twist.twist.linear.z
        s.rpy_odo = _quat_to_rpy(msg.pose.pose.orientation)
        # КОРРЕКЦИЯ фильтра Калмана абсолютными углами одометрии
        for i in range(3):
            self.kf[i].update(s.rpy_odo[i])

    # ---------- основной апдейт ----------
    def update(self, target: List[float], dt: float):
        s = self.state
        # Итоговая ориентация: оценка фильтра Калмана (если запущен), иначе одометрия
        if self.kf[0].init:
            s.rpy = [self.kf[0].x, self.kf[1].x, self.kf[2].x]
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
        # Угловые скорости: с IMU — гироскоп (чисто), иначе численная производная
        if s.imu_ok:
            s.roll_d = s.gyro[0]
            s.pitch_d = s.gyro[1]
            s.yaw_d = s.gyro[2]
        else:
            s.roll_d = (s.rpy[0] - self._pr[0]) / dt
            s.pitch_d = (s.rpy[1] - self._pr[1]) / dt
            s.yaw_d = (s.rpy[2] - self._pr[2]) / dt
        self._pr = list(s.rpy)
