import re
import os
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches

PHASE_COLORS = {
    'Круиз': '#add8e6',
    'Сближение': '#90ee90',
    'Корр.высоты': '#ffffe0',
    'Коридор-Z': '#e0ffff',
    'Стабилизация': '#ffb6c1',
    'Готово': '#d3d3d3',
    'AVOID': '#ffa07a',
    'Торможение': '#f08080'
}

def parse_log(filename):
    data = {
        'time': [], 'wp': [], 'wp_total': [], 'phase': [],
        'x': [], 'y': [], 'z': [], 'v': [], 'vz': [],
        'dist_2d': [], 'dz': [],
        'roll': [], 'pitch': [], 'yaw': [],
        'rud_v': [], 'rud_h_l': [], 'rud_h_r': [],
        'ballast': [],
        'target_z': []
    }
    
    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        
    t = 0
    for line in lines:
        if line.startswith("===") or "Точка" in line or not line.strip() or "ФАЗА" in line or "Миссия завершена" in line:
            continue
            
        clean_line = re.sub(r'\x1b\[.*?m', '', line).replace('\r', '').strip()
        if not clean_line.startswith("WP"): continue
        
        try:
            wp_match = re.search(r'WP (\d+)/(\d+)', clean_line)
            if wp_match:
                data['wp'].append(int(wp_match.group(1)))
                data['wp_total'].append(int(wp_match.group(2)))
            
            phase_match = re.search(r'WP \d+/\d+\s+([A-Za-zА-Яа-я0-9_-]+)', clean_line)
            if phase_match:
                data['phase'].append(phase_match.group(1).strip())
                
            xyz_match = re.search(r'X:\s*([+-]?\d+\.?\d*)\s*Y:\s*([+-]?\d+\.?\d*)\s*Z:\s*([+-]?\d+\.?\d*)', clean_line)
            if xyz_match:
                data['x'].append(float(xyz_match.group(1)))
                data['y'].append(float(xyz_match.group(2)))
                data['z'].append(float(xyz_match.group(3)))
                
            v_match = re.search(r'V:\s*([+-]?\d+\.?\d*)\s*Vz:\s*([+-]?\d+\.?\d*)', clean_line)
            if v_match:
                data['v'].append(float(v_match.group(1)))
                data['vz'].append(float(v_match.group(2)))
                
            d_match = re.search(r'D:\s*([+-]?\d+\.?\d*)м\s*dZ:\s*([+-]?\d+\.?\d*)', clean_line)
            if d_match:
                data['dist_2d'].append(float(d_match.group(1)))
                data['dz'].append(float(d_match.group(2)))
                data['target_z'].append(data['z'][-1] - data['dz'][-1])
                
            rpy_match = re.search(r'RPY:\s*([+-]?\d+\.?\d*)/([+-]?\d+\.?\d*)/([+-]?\d+\.?\d*)°', clean_line)
            if rpy_match:
                data['roll'].append(float(rpy_match.group(1)))
                data['pitch'].append(float(rpy_match.group(2)))
                data['yaw'].append(float(rpy_match.group(3)))
                
            rud_match = re.search(r'руль в:\s*([+-]?\d+\.?\d*)°\s*гор:\s*([+-]?\d+\.?\d*)/([+-]?\d+\.?\d*)°', clean_line)
            if rud_match:
                data['rud_v'].append(float(rud_match.group(1)))
                data['rud_h_l'].append(float(rud_match.group(2)))
                data['rud_h_r'].append(float(rud_match.group(3)))
            else:
                data['rud_v'].append(0.0)
                data['rud_h_l'].append(0.0)
                data['rud_h_r'].append(0.0)
                
            bal_match = re.search(r'B:\s*(\d+)%', clean_line)
            if bal_match:
                data['ballast'].append(float(bal_match.group(1)))
            else:
                data['ballast'].append(50.0)
                
            # Переводим тики (строки) в секунды. 
            # Телеметрия пишется раз в 0.15 сек (из telemetry.py: if t - self._t < 0.15: return)
            data['time'].append(t * 0.15)
            t += 1
            
        except Exception:
            pass
            
    return data

def parse_world(world_path):
    obstacles = []
    if not world_path or not os.path.exists(world_path):
        return obstacles
    with open(world_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    model_blocks = re.findall(r'<model name="([^"]+)">(.*?)</model>', content, re.DOTALL)
    for name, block in model_blocks:
        pose_match = re.search(r'<pose>([^<]+)</pose>', block)
        if not pose_match: continue
        
        pose_parts = list(map(float, pose_match.group(1).split()))
        x, y, z, roll, pitch, yaw = pose_parts
        
        box_match = re.search(r'<box>\s*<size>([^<]+)</size>\s*</box>', block)
        if box_match:
            sx, sy, sz = list(map(float, box_match.group(1).split()))
            obstacles.append({'type': 'box', 'name': name, 'x': x, 'y': y, 'yaw': yaw, 'sx': sx, 'sy': sy})
            continue
            
        cyl_match = re.search(r'<(?:cylinder|sphere)>\s*<radius>([^<]+)</radius>', block)
        if cyl_match:
            r = float(cyl_match.group(1))
            obstacles.append({'type': 'circle', 'name': name, 'x': x, 'y': y, 'r': r})
            
    return obstacles

def add_phase_background(ax, time, phases):
    if not time: return
    start_idx = 0
    curr_phase = phases[0]
    added_labels = set()
    
    for i in range(1, len(time)):
        if phases[i] != curr_phase or i == len(time) - 1:
            color = PHASE_COLORS.get(curr_phase, '#ffffff')
            label = f"Фаза: {curr_phase}" if curr_phase not in added_labels else None
            added_labels.add(curr_phase)
            ax.axvspan(time[start_idx], time[i], facecolor=color, alpha=0.3, label=label)
            curr_phase = phases[i]
            start_idx = i

def save_time_series_plot(title, ylabel, time, lines_data, phases, filename, invert_y=False):
    fig, ax = plt.subplots(figsize=(10, 6))
    add_phase_background(ax, time, phases)
    
    for label, (y_arr, color, linestyle) in lines_data.items():
        ax.plot(time, y_arr, label=label, linewidth=2, color=color, linestyle=linestyle)
        
    if invert_y:
        ax.invert_yaxis()
        
    ax.set_title(title)
    ax.set_xlabel('Время (секунды)')
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle='--', alpha=0.6)
    
    ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), borderaxespad=0.)
    plt.tight_layout(pad=3.0)
    plt.savefig(filename, dpi=150)
    plt.close()

def plot_data(data, out_dir="graphs", world_path=None):
    if not data['time']:
        print("Данные не найдены!")
        return

    # --- ОТСЧЕТ С 5-го ТИКА ---
    # Обрезаем все списки данных, чтобы убрать начальные переходные процессы
    for key in data:
        if isinstance(data[key], list):
            data[key] = data[key][5:]

    os.makedirs(out_dir, exist_ok=True)
    time = data['time']
    phases = data['phase']

    if not time:
        print("Недостаточно данных для построения графиков (менее 5 тиков).")
        return

    print(f"Построение графиков в папку '{os.path.abspath(out_dir)}'...")

    # --- 1. 3D Траектория ---
    fig = plt.figure(figsize=(12, 10))
    ax3d = fig.add_subplot(111, projection='3d')
    
    ax3d.plot(data['x'], data['y'], data['z'], color='gray', alpha=0.5, zorder=1, label='Путь')
    
    for phase_name in set(phases):
        idx = [i for i, p in enumerate(phases) if p == phase_name]
        x_p = [data['x'][i] for i in idx]
        y_p = [data['y'][i] for i in idx]
        z_p = [data['z'][i] for i in idx]
        ax3d.scatter(x_p, y_p, z_p, color=PHASE_COLORS.get(phase_name, '#000000'), 
                   label=f"Фаза: {phase_name}", zorder=2, s=15)
                   
    # Обновленные цели для основного сценария ВКР
    targets_x = [30.0, 30.0, 0.0]
    targets_y = [0.0, 22.0, 0.0]
    targets_z = [-3.0, -7.0, 5.0]
    
    ax3d.scatter(targets_x, targets_y, targets_z, marker='*', s=300, color='gold', edgecolors='black', label='Цели (Waypoints)', zorder=3)
    
    # Добавляем порядковые номера точек
    for i in range(len(targets_x)):
        ax3d.text(targets_x[i], targets_y[i], targets_z[i] + 0.5, str(i+1), 
                  fontsize=12, fontweight='bold', color='black', zorder=4)
    
    # --- ОТОБРАЖЕНИЕ ОБЪЕКТОВ ИЗ МИРА ---
    if world_path:
        obstacles = parse_world(world_path)
        # Считаем границы траектории, чтобы фильтровать далекие объекты
        min_x, max_x = min(data['x']), max(data['x'])
        min_y, max_y = min(data['y']), max(data['y'])
        margin = 20.0 # Показываем объекты в радиусе 20м от пути
        
        for obj in obstacles:
            # Фильтруем: рисуем только те, что рядом с путем
            if not (min_x - margin <= obj['x'] <= max_x + margin and 
                    min_y - margin <= obj['y'] <= max_y + margin):
                continue

            color = 'red' if 'demo' in obj['name'] or 'test' in obj['name'] else 'gray'
            alpha = 0.6 if 'demo' in obj['name'] or 'test' in obj['name'] else 0.3
            
            if obj['type'] == 'circle':
                # Рисуем круг (как набор точек)
                import numpy as np
                theta = np.linspace(0, 2*np.pi, 20)
                cx = obj['x'] + obj['r'] * np.cos(theta)
                cy = obj['y'] + obj['r'] * np.sin(theta)
                ax3d.plot(cx, cy, 0, color=color, alpha=alpha, linewidth=2)
            elif obj['type'] == 'box':
                # Рисуем контур прямоугольника
                sx, sy = obj.get('sx', 1.0)/2, obj.get('sy', 1.0)/2
                bx = [obj['x']-sx, obj['x']+sx, obj['x']+sx, obj['x']-sx, obj['x']-sx]
                by = [obj['y']-sy, obj['y']-sy, obj['y']+sy, obj['y']+sy, obj['y']-sy]
                ax3d.plot(bx, by, 0, color=color, alpha=alpha, linewidth=2)
                
    coord_lines = ["Координаты целей (X, Y, Z):"]
    for i in range(len(targets_x)):
        coord_lines.append(f"{i+1}: ({targets_x[i]}, {targets_y[i]}, {targets_z[i]})")
        
    ax3d.text2D(0.02, 0.05, "\n".join(coord_lines), transform=ax3d.transAxes, fontsize=10,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.8))

    ax3d.set_title('3D Траектория аппарата (Спиральный подъем)')
    ax3d.set_xlabel('Ось X (м)')
    ax3d.set_ylabel('Ось Y (м)')
    ax3d.set_zlabel('Ось Z (м)')
    
    ax3d.view_init(elev=20, azim=45)
    ax3d.set_box_aspect(aspect=(1, 1, 0.7))
    
    ax3d.legend(loc='upper right', bbox_to_anchor=(1.2, 1.0))
    plt.tight_layout(pad=3.0)
    plt.savefig(os.path.join(out_dir, '01_Траектория_3D.png'), dpi=150, bbox_inches='tight')
    plt.close()

    # --- 2. Глубина БЕЗ ошибки и без перевернутой оси ---
    # По вашей просьбе: "глубина вниз а высота вверх"
    # Это значит, что положительные значения (высота) должны быть НАВЕРХУ графика,
    # а отрицательные значения (глубина) - ВНИЗУ графика. 
    # Значит, ось инвертировать НЕ НАДО (invert_y=False), так как обычные декартовы оси 
    # именно так и работают (плюс вверху, минус внизу).
    save_time_series_plot(
        'График удержания глубины', 'Ось Z (м)', time, 
        {
            'Целевая глубина': (data['target_z'], 'green', '--'),
            'Фактическая глубина': (data['z'], 'blue', '-')
        }, 
        phases, os.path.join(out_dir, '02_Глубина.png'), invert_y=False
    )

    # --- 3. Скорости ---
    save_time_series_plot(
        'Скорости аппарата', 'Скорость (м/с)', time, 
        {'Скорость вперед (V)': (data['v'], 'green', '-'), 'Вертикальная скорость (Vz)': (data['vz'], 'purple', '-')}, 
        phases, os.path.join(out_dir, '03_Скорости.png')
    )

    # --- 4. Дистанция ---
    save_time_series_plot(
        'Дистанция до целевой точки', 'Дистанция (м)', time, 
        {'2D Дистанция': (data['dist_2d'], 'black', '-')}, 
        phases, os.path.join(out_dir, '04_Дистанция.png')
    )

    # --- 5. Балласт ---
    save_time_series_plot(
        'Команды управления балластом', 'Объем (%)', time, 
        {'Балласт': (data['ballast'], 'cyan', '-')}, 
        phases, os.path.join(out_dir, '05_Балласт.png')
    )

    # --- 6. Крен и Тангаж (вместе) ---
    save_time_series_plot(
        'Ориентация: Крен и Тангаж', 'Градусы (°)', time, 
        {'Крен (Roll)': (data['roll'], 'red', '-'), 'Тангаж (Pitch)': (data['pitch'], 'green', '-')}, 
        phases, os.path.join(out_dir, '06_Крен_и_Тангаж_Градусы.png')
    )
    
    # --- 7. Рыскание (отдельно) ---
    save_time_series_plot(
        'Ориентация: Курс (Рыскание)', 'Градусы (°)', time, 
        {'Рыскание (Yaw)': (data['yaw'], 'blue', '-')}, 
        phases, os.path.join(out_dir, '07_Рыскание_Градусы.png')
    )

    print("Все графики успешно сохранены!")

if __name__ == '__main__':
    import sys
    log_file = sys.argv[1] if len(sys.argv) > 1 else 'autopilot_full.log'
    if not os.path.exists(log_file):
        print(f"Ошибка: Файл '{log_file}' не найден!")
    else:
        data = parse_log(log_file)
        world_f = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../my_auv_bringup/worlds/static_world.sdf')
        if not os.path.exists(world_f):
            world_f = None
        plot_data(data, out_dir="graphs", world_path=world_f)
