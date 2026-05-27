#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import threading
import subprocess

class BallastController(Node):
    def __init__(self):
        super().__init__('ballast_control')
        self.world = "static_world"
        self.model = "submarine"
        self.link = "body"
        self.link_full = f"{self.model}::{self.link}"

        # Пока один бак в центре масс.
        self.force = 0.0

        # Таймер публикации 20 Гц
        self.timer = self.create_timer(0.05, self.publish_force)
        self.get_logger().info("Балласт: введите силу (Н), например -500 для погружения")
        threading.Thread(target=self.input_loop, daemon=True).start()

    def publish_force(self):
        if self.force == 0.0:
            return
        # ТОЧНО ТАКАЯ ЖЕ СТРОКА, КАК В РАБОТАЮЩЕЙ РУЧНОЙ КОМАНДЕ
        # Только добавляем публикацию в persistent топик
        cmd = [
            "gz", "topic", "-t", f"/world/{self.world}/wrench/persistent",
            "-m", "gz.msgs.EntityWrench",
            "-p", f'entity: {{name: "{self.link_full}", type: LINK}}, wrench: {{force: {{z: {self.force}}}}}'
        ]
        # Раскомментируйте следующую строку для отладки (будет видна команда)
        # self.get_logger().info(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def input_loop(self):
        while rclpy.ok():
            line = sys.stdin.readline().strip()
            if not line:
                continue
            try:
                val = float(line)
                self.force = val
                self.get_logger().info(f"Сила установлена: {val} Н")
            except ValueError:
                self.get_logger().info("Введите число (сила в Ньютонах)")

def main(args=None):
    rclpy.init(args=args)
    node = BallastController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
