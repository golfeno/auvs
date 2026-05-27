#!/usr/bin/env python3
"""AUV Test Straight | Clean 5-Second Telemetry Profile"""
import rclpy, math, time, sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64

P_Z0 = 101325.0
RHO_G = 9810.0

class AUVTestStraight(Node):
    def __init__(self):
        super().__init__('auv_test_straight')
        
        # Подписки на датчики и одометрию
        self.create_subscription(Odometry, '/model/submarine/odometry', self.odom_cb, 10)
        self.create_subscription(Float32, '/model/submarine/pressure', self.press_cb, 10)
        
        # Подписки на топики силы моторов для вывода в лог
        self.create_subscription(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', self.lt_cb, 10)
        self.create_subscription(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', self.rt_cb, 10)

        # Переменные состояния телеметрии
        self.pos = [0.0, 0.0, 0.0]
        self.raw_press = 0.0
        self.vel = 0.0
        self.prev_vel = 0.0
        self.accel = 0.0
        self.rpy = [0.0, 0.0, 0.0]
        self.mot_lt = 0.0
        self.mot_rt = 0.0
        
        self.start_time = time.time()
        self.last_log_time = time.time()
        
        # 🔥 Жесткий таймер вывода — строго раз в 5.0 секунд, никакого спама каждую секунду!
        self.create_timer(5.0, self.log_telemetry_5sec)
        
        print("\n" + "="*80)
        print("📋 Модуль TEST STRAIGHT запущен | Мониторинг каждые 5 секунд")
        print("="*80 + "\n")

    def press_cb(self, msg):
        self.raw_press = msg.data
        # Защита балласта: исключаем отрицательное давление на поверхности симулятора
        safe_press = max(0.0, msg.data)
        self.pos[2] = (P_Z0 - safe_press) / RHO_G

    def lt_cb(self, msg): self.mot_lt = msg.data
    def rt_cb(self, msg): self.mot_rt = msg.data

    def odom_cb(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.vel = msg.twist.twist.linear.x
        
        q = msg.pose.pose.orientation
        self.rpy[0] = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        self.rpy[1] = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        self.rpy[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))

    def log_telemetry_5sec(self):
        now = time.time()
        dt = now - self.last_log_time
        elapsed = now - self.start_time

        # Честное среднее ускорение за 5-секундный отрезок времени
        self.accel = (self.vel - self.prev_vel) / dt if dt > 0 else 0.0
        self.prev_vel = self.vel
        self.last_log_time = now

        # Расчет расстояния до целевой точки [40.0, 40.0] строго по горизонтальной плоскости X-Y
        dist_xy = math.hypot(40.0 - self.pos[0], 40.0 - self.pos[1])

        # Вывод структурированного блока параметров
        print(f"⏱️  [ Ротация лога: T + {elapsed:.1f} сек ] " + "-"*50)
        print(f"   📍 Текущие координаты ::  X = {self.pos[0]:+6.2f} м  |  Y = {self.pos[1]:+6.2f} м  |  Z = {self.pos[2]:6.2f} м")
        print(f"   📏 Дистанция по XY    ::  D2D до цели = {dist_xy:.2f} метров")
        print(f"   💨 Скорость и Ускорение::  V = {self.vel:+.3f} м/с  |  Acc = {self.accel:+.3f} м/с²")
        print(f"   📐 Пространственные углы::  Roll: {math.degrees(self.rpy[0]):+5.1f}° | Pitch: {math.degrees(self.rpy[1]):+5.1f}° | Yaw: {math.degrees(self.rpy[2]):+5.1f}°")
        print(f"   ⚙️  Силовая установка  ::  Мотор Л: {self.mot_lt:+6.1f} N | Мотор П: {self.mot_rt:+6.1f} N | P_Raw: {self.raw_press:+.1f} Pa")
        print("-" * 88)

def main():
    rclpy.init()
    node = AUVTestStraight()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
