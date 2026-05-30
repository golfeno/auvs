#!/usr/bin/env python3
"""ДЕМО-автопилот: те же точки/вывод, но режим глубины МЕНЯЕТСЯ по сегментам.

Расписание (как просил пользователь):
    участок 0→1 точки:  БАКИ + РУЛИ   (DepthMode.BOTH)
    участок 1→2 точки:  только БАКИ   (DepthMode.BALLAST)
    участок 2→3 точки:  только РУЛИ   (DepthMode.RUDDER)

Точки берутся из обычного ~/auv/waypoints.txt (те же, что и у боевого автопилота).
Вывод — та же одна перезаписываемая строка телеметрии, плюс в ней видно текущий
режим глубины с пометкой «(демо)».

Запуск:  ros2 run my_auv_control autopilot_demo
"""
import os
import rclpy

from .autopilot_node import AUVAutopilotNode, load_waypoints
from .models import MotorMode, DepthMode, ActuatorCommands
from .version import VERSION, BUILD_NUMBER


# Расписание режима глубины по индексу сегмента (wp_idx):
#   index = текущая цель-1: 0 -> идём к точке 1, 1 -> к точке 2, 2 -> к точке 3
DEPTH_SCHEDULE = [
    DepthMode.BOTH,      # 0→1: баки + рули
    DepthMode.BALLAST,   # 1→2: только баки
    DepthMode.RUDDER,    # 2→3: только рули
]


def main():
    print("=" * 60)
    print(f"  AUV ДЕМО {VERSION} #{BUILD_NUMBER} | режим глубины меняется по сегментам")
    print("    0→1: БАКИ+РУЛИ    1→2: только БАКИ    2→3: только РУЛИ")
    print("=" * 60)

    default_file = os.path.expanduser("~/auv/waypoints.txt")
    if not os.path.exists(default_file):
        print(f"  НЕТ файла точек: {default_file}")
        return
    wps = load_waypoints(default_file)
    print(f"  Загружено {len(wps)} точек:")
    for i, w in enumerate(wps, 1):
        print(f"    {i}: ({w[0]}, {w[1]}, {w[2]})")
    if len(wps) < 2:
        print("  Нужно >=2 точек для демо.")
        return

    # Двигатели — дифференциал (как обычно). Стартовый режим глубины = первый из
    # расписания; дальше переключается автоматически в _apply_segment_depth().
    motor = MotorMode.DUAL
    start_depth = DEPTH_SCHEDULE[0]

    rclpy.init()
    node = AUVAutopilotNode(wps, motor, start_depth)
    node.depth_schedule = DEPTH_SCHEDULE      # включаем демо-расписание
    node.tl.depth_label = 'Глубина:БАКИ+РУЛИ (демо)'
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node._pub(ActuatorCommands())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
