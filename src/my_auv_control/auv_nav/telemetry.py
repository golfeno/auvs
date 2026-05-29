"""Telemetry (v51.0) — single updating line."""
import sys, math
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from .models import VehicleState, PHASE_TR, ActuatorCommands


class Telemetry:
    def __init__(self, node):
        self.n = node
        self.pub_pos = node.create_publisher(Point, '/auv/position', 10)
        self.pub_status = node.create_publisher(String, '/auv/status', 10)
        self._t = 0.0
        self._pv = 0.0; self._pz = 0.0
        self._ax = 0.0; self._az = 0.0

    def analytics(self, s, phase, wp_idx):
        try:
            p = Point()
            p.x = float(s.pos[0]); p.y = float(s.pos[1]); p.z = float(s.pos[2])
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
            f"\r\033[K"
            f"WP {wp_idx+1}/{tw} {ru:<10s} "
            f"X:{s.pos[0]:+6.1f} Y:{s.pos[1]:+6.1f} Z:{s.pos[2]:+5.2f} "
            f"V:{s.vel:+.2f} Vz:{s.dz_dt:+.2f} "
            f"D:{s.dist_2d:4.1f}m dZ:{s.z_err:+.2f} "
            f"R:{rd:+.0f}°P:{pd:+.0f}°Y:{yd:+.0f}°"
        )

        if cmd and hasattr(cmd, 'ballast_volume') and cmd.ballast_volume != 0.5:
            line += f" B:{cmd.ballast_volume:.0%}"

        sys.stdout.write(line)
        sys.stdout.flush()

    def log_wp(self, wi):
        sys.stdout.write("\n  ✓ Точка " + str(wi))
        sys.stdout.flush()
