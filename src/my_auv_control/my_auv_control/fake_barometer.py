#!/usr/bin/env python3
"""Virtual Barometer vFinal: Flexible P_Z0 Reference | Reads Gazebo Raw Pose"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import subprocess
import re
import threading

# 🔥 ОПОРНОЕ ДАВЛЕНИЕ НА УРОВНЕ МИРА Z=0
# Если Z=0 находится под водой, увеличь это значение: P_Z0 = 101325 + 9810 * |глубина_нуля|
P_Z0 = 101325.0
RHO_G = 9810.0

class DirectGzBarometer(Node):
    def __init__(self):
        super().__init__('virtual_barometer')
        self.pub = self.create_publisher(Float32, '/model/submarine/pressure', 10)
        self.running = True
        
        self.gz_proc = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', '/world/static_world/dynamic_pose/info'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1
        )
        
        self.thread = threading.Thread(target=self._read_gz_stream, daemon=True)
        self.thread.start()
        self.get_logger().info(f"📡 Читаю сырую физику. P_Z0={P_Z0:.0f} Pa")

    def _read_gz_stream(self):
        is_sub = False
        try:
            for line in self.gz_proc.stdout:
                if not self.running: break
                line = line.strip()
                if 'name:' in line and 'submarine' in line:
                    is_sub = True
                elif is_sub and 'z:' in line:
                    match = re.search(r'z:\s*([-\d.eE+]+)', line)
                    if match:
                        z = float(match.group(1))
                        # 🔥 Гибкая формула: давление падает при росте Z, растёт при падении Z
                        pressure = P_Z0 - (RHO_G * z)
                        self.pub.publish(Float32(data=float(pressure)))
                        if not hasattr(self, 'ok'):
                            self.get_logger().info(f"✅ ДАВЛЕНИЕ ПОШЛО! Z={z:+.2f}m -> P={pressure:.0f}Pa")
                            self.ok = True
                    is_sub = False
        except Exception as e:
            self.get_logger().error(f"Ошибка потока: {e}")

    def on_shutdown(self):
        self.running = False
        if self.gz_proc: self.gz_proc.terminate()

def main():
    rclpy.init()
    node = DirectGzBarometer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.on_shutdown(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
