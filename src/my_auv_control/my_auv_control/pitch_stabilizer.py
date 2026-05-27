#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import subprocess
import threading
import math

class PitchStabilizer(Node):
    def __init__(self):
        super().__init__('pitch_stabilizer')
        self.world = "static_world"
        self.link_name = "submarine::body"
        self.active = False
        self.target_pitch = 0.0
        self.current_pitch = 0.0

        # Координаты баков (носовая и кормовая пары)
        self.bow_left   = {'x':  0.40, 'y':  0.20, 'z': -0.167}
        self.bow_right  = {'x':  0.40, 'y': -0.20, 'z': -0.167}
        self.stern_left = {'x': -0.40, 'y':  0.20, 'z': -0.167}
        self.stern_right= {'x': -0.40, 'y': -0.20, 'z': -0.167}

        # Плечо момента (расстояние между носовой и кормовой парами)
        self.arm = 0.8   # 0.4 - (-0.4) = 0.8 м

        # ПИД-коэффициенты (подберите под свой аппарат)
        self.kp = 1500.0
        self.ki = 30.0
        self.kd = 150.0
        self.integral = 0.0
        self.prev_error = 0.0

        # Подписка на IMU
        self.imu_sub = self.create_subscription(Imu, '/model/submarine/imu', self.imu_cb, 10)

        # Таймер управления (20 Гц)
        self.timer = self.create_timer(0.05, self.control_loop)

        # Поток для ввода с клавиатуры
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

        self.get_logger().info("Pitch stabilizer ready. Press 'p' to toggle, 'c' nose up, 'v' nose down.")

    def imu_cb(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.current_pitch = math.asin(sinp)

    def control_loop(self):
        if not self.active:
            self.apply_force_all(0.0)
            return

        dt = 0.05
        error = self.target_pitch - self.current_pitch
        self.integral += error * dt
        self.integral = max(min(self.integral, 10.0), -10.0)
        derivative = (error - self.prev_error) / dt
        moment = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Момент в пару сил: F = момент / плечо
        force = moment / self.arm
        force = max(min(force, 200.0), -200.0)

        # Носовые баки получают силу force, кормовые – -force
        self.apply_force(self.bow_left, force)
        self.apply_force(self.bow_right, force)
        self.apply_force(self.stern_left, -force)
        self.apply_force(self.stern_right, -force)

        # Лог для отладки
        # self.get_logger().debug(f"pitch={self.current_pitch:.3f}, err={error:.3f}, f={force:.1f}")

    def apply_force_to_tank(self, tank_name, force):
        if abs(force) < 0.1:
            return
        cmd = [
            "gz", "topic", "-t", f"/world/{self.world}/wrench/persistent",
            "-m", "gz.msgs.EntityWrench",
            "-p", f'entity: {{name: "submarine::{tank_name}", type: LINK}}, reference_frame: "world", wrench: {{force: {{z: {force}}}}}'
        ]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def apply_force_all(self, f):
        self.apply_force(self.bow_left, f)
        self.apply_force(self.bow_right, f)
        self.apply_force(self.stern_left, f)
        self.apply_force(self.stern_right, f)

    def keyboard_loop(self):
        while rclpy.ok():
            try:
                key = input().strip().lower()
                if key == 'p':
                    self.active = not self.active
                    if not self.active:
                        self.integral = 0.0
                        self.prev_error = 0.0
                    self.get_logger().info(f"Stabilization {'ON' if self.active else 'OFF'}")
                elif key == 'c' and self.active:
                    self.target_pitch += 0.05
                    self.target_pitch = min(self.target_pitch, 0.35)
                    self.get_logger().info(f"Target pitch: {self.target_pitch:.2f} rad")
                elif key == 'v' and self.active:
                    self.target_pitch -= 0.05
                    self.target_pitch = max(self.target_pitch, -0.35)
                    self.get_logger().info(f"Target pitch: {self.target_pitch:.2f} rad")
                elif key == 'r':
                    self.target_pitch = 0.0
                    self.integral = 0.0
                    self.prev_error = 0.0
                    self.get_logger().info("Target pitch reset to 0")
                elif key == 'q':
                    break
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = PitchStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
