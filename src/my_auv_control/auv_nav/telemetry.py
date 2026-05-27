"""Telemetry (v50.27) — robust publishing, single line."""
import sys, math
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from .models import VehicleState, PHASE_TR


class Telemetry:
    def __init__(self, node, dm=None):
        self.n = node
        self.pub_pos = node.create_publisher(Point, '/auv/position', 10)
        self.pub_status = node.create_publisher(String, '/auv/status', 10)
        self._t = 0.0
        self._pv = 0.0; self._pz = 0.0
        self._ax = 0.0; self._az = 0.0

    def analytics(self, s, phase, wp_idx):
        try:
            p = Point()
            p.x = float(s.pos[0])
            p.y = float(s.pos[1])
            p.z = float(s.pos[2])
            self.pub_pos.publish(p)
            m = String()
            m.data = str(wp_idx+1) + "_" + str(phase)
            self.pub_status.publish(m)
        except Exception:
            pass

    def log(self, s, phase, wp_idx, t, tw, cmd=None):
        if t - self._t < 0.15:
            return
        dt = t - self._t if self._t > 0 else 0.15
        self._t = t
        self._ax = (s.vel - self._pv) / dt if dt > 0.01 else 0.0
        self._az = (s.dz_dt - self._pz) / dt if dt > 0.01 else 0.0
        self._pv = s.vel; self._pz = s.dz_dt

        ru = PHASE_TR.get(phase, str(phase))
        rd = math.degrees(s.rpy[0])
        pd = math.degrees(s.rpy[1])
        yd = math.degrees(s.rpy[2])

        line = (
            "\r\033[K"
            "WP " + str(wp_idx+1) + "/" + str(tw) + " " + ru + " "
            "X:" + format(s.pos[0], '+6.1f') + " Y:" + format(s.pos[1], '+6.1f') + " Z:" + format(s.pos[2], '+5.2f') + " "
            "V:" + format(s.vel, '+.2f') + " Vz:" + format(s.dz_dt, '+.2f') + " "
            "Ax:" + format(self._ax, '+.1f') + " Az:" + format(self._az, '+.1f') + " "
            "D:" + format(s.dist_2d, '4.1f') + "m dZ:" + format(s.z_err, '+.2f') + " "
            "R:" + format(rd, '+.0f') + " P:" + format(pd, '+.0f') + " Y:" + format(yd, '+.0f')
        )

        sys.stdout.write(line)
        sys.stdout.flush()

    def log_wp(self, wi):
        sys.stdout.write("\n  ✓ Точка " + str(wi))
        sys.stdout.flush()
