#!/usr/bin/env python3
"""AUV Test Straight v5: Full Diagnostics (Pressure, Depth, Ballast Forces)"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

class TestStraight(Node):
    def __init__(self):
        super().__init__('auv_test_straight')
        self.create_subscription(Odometry, '/model/submarine/odometry', self.odom_cb, 10)
        qos_s = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32, '/model/submarine/pressure', self.press_cb, qos_s)
        
        self.raw_pressure = 101325.0
        self.depth = 0.0
        self.use_pressure = False
        self.pos = [0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        self.timer = self.create_timer(1.0, self.print_status)
        self.get_logger().info("🚀 Test Straight v5 Started (Diagnostics Mode)")

    def press_cb(self, msg):
        self.raw_pressure = msg.data
        self.depth = -((self.raw_pressure - 101325.0) / 9810.0)
        self.use_pressure = True

    def odom_cb(self, msg):
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        q = msg.pose.pose.orientation
        self.rpy[0] = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        self.rpy[1] = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        self.rpy[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))

    def print_status(self):
        src = 'P' if self.use_pressure else 'O'
        target_z = -3.0
        depth_err = self.depth - target_z
        Kp = 90.0
        force_per_ballast = -(Kp * depth_err)
        
        print(f"[STATUS] Pos:[{self.pos[0]:+.1f}, {self.pos[1]:+.1f}, {self.depth:+.2f}m ({src})]")
        print(f"         Raw_P: {self.raw_pressure:.1f} Pa | Err_Z: {depth_err:+.2f}m")
        print(f"         Ballast_Force (EACH): {force_per_ballast:+.1f} N")
        print("-" * 60)

def main():
    rclpy.init()
    node = TestStraight()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
