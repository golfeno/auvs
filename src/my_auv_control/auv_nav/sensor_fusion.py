"""Sensor Fusion (v50.4)"""
import math
from typing import List
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from .models import VehicleState, Phys


class SensorFusion:
    def __init__(self, node: Node):
        self.node = node
        self.state = VehicleState()
        self._pb = 0.0
        self._pr = [0.0, 0.0, 0.0]
        self.node.create_subscription(Odometry, '/model/submarine/odometry', self._odom, 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.node.create_subscription(Float32, '/model/submarine/pressure', self._press, qos)

    def _press(self, msg):
        self.state.baro_z = (Phys.P_Z0 - msg.data) / Phys.RHO_G

    def _odom(self, msg):
        s = self.state
        s.pos[0] = msg.pose.pose.position.x
        s.pos[1] = msg.pose.pose.position.y
        s.pos[2] = self.state.baro_z
        s.vel = msg.twist.twist.linear.x
        s.vel_z = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        s.rpy[0] = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        s.rpy[1] = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        s.rpy[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))

    def update(self, target: List[float], dt: float):
        s = self.state
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
        s.roll_d = (s.rpy[0] - self._pr[0]) / dt
        s.pitch_d = (s.rpy[1] - self._pr[1]) / dt
        s.yaw_d = (s.rpy[2] - self._pr[2]) / dt
        self._pr = list(s.rpy)
