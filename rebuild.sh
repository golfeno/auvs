#!/usr/bin/env bash
# Чистая пересборка воркспейса AUV.
# Главная причина "правки не применяются": colcon для ament_python и data_files
# (SDF/launch/rviz/yaml) КОПИРУЕТ файлы в install/. Без пересборки запускается старое.
#
# --symlink-install делает симлинки вместо копий → правки Python/конфигов
# подхватываются без повторной сборки (модели/launch всё равно лучше пересобирать).
set -e
cd "$(dirname "$0")"

echo ">>> Останавливаю возможные запущенные ноды..."
pkill -f parameter_bridge 2>/dev/null || true
pkill -f 'ros_gz_sim'      2>/dev/null || true

echo ">>> Удаляю старые install/ build/ (гарантия отсутствия устаревших копий)..."
rm -rf build install log

echo ">>> colcon build --symlink-install ..."
colcon build --symlink-install

echo ""
echo ">>> Готово. Теперь в КАЖДОМ терминале выполни:"
echo "    source ~/auv/install/setup.bash"
echo ">>> Запуск:"
echo "    ros2 launch my_auv_bringup simulation.launch.py"
echo "    ros2 run my_auv_control autopilot"
echo ""
echo ">>> При старте автопилота должно печататься: 'AUV Autopilot v53.0'."
echo "    Если видишь 'v51.4' — значит запущен старый install (не сделал source/rebuild)."
