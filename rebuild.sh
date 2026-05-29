#!/usr/bin/env bash
# Чистая пересборка воркспейса AUV + АВТО-ИНКРЕМЕНТ номера сборки.
#
# Зачем номер: colcon для ament_python и data_files (SDF/launch/rviz) КОПИРУЕТ
# файлы в install/. Без пересборки запускается старое. Номер сборки печатается
# в баннере автопилота — всегда видно, что запущена свежая сборка.
#
# --symlink-install: правки Python подхватываются без повторной сборки.
set -e
cd "$(dirname "$0")"

VER_FILE="src/my_auv_control/auv_nav/version.py"

# --- инкремент BUILD_NUMBER ---
CUR=$(grep -oP 'BUILD_NUMBER\s*=\s*\K[0-9]+' "$VER_FILE")
NEW=$((CUR + 1))
sed -i "s/^BUILD_NUMBER = .*/BUILD_NUMBER = $NEW/" "$VER_FILE"
echo ">>> Номер сборки: $CUR -> $NEW"

echo ">>> Останавливаю запущенные ноды..."
pkill -f parameter_bridge 2>/dev/null || true
pkill -f 'ros_gz_sim'      2>/dev/null || true

echo ">>> Чищу install/ build/ ..."
rm -rf build install log

echo ">>> colcon build --symlink-install ..."
colcon build --symlink-install

echo ""
echo ">>> ГОТОВО. Сборка #$NEW. В каждом терминале:  source ~/auv/install/setup.bash"
echo ">>> Запуск:"
echo "    ros2 launch my_auv_bringup simulation.launch.py"
echo "    ros2 run my_auv_control autopilot"
echo ">>> При старте баннер должен показать: 'СБОРКА #$NEW'. Иначе запущен старый install."
