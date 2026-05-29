#!/usr/bin/env python3
"""Mass Calibration Tool (v51.0) — параметризован по модели, актуальные константы."""
import sys, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

P_Z0 = 101325.0
RHO_G = 9810.0
DURATION = 120.0

# Параметры моделей (актуальные SDF)
MODELS = {
    'submarine': {
        'mass': 124.18,          # корпус 107.5 + 4 балласта (5.88+5.25+3.0+2.554)
        'hull_radius': 0.17,
        'hull_length': 1.5,
        'ballast': (0.375, 0.05, 0.05),  # размер одного бака (одинаковые, по всему килю)
        'n_ballast': 4,
    },
    'submarine_nb': {
        'mass': 121.8,           # корпус (без балластов)
        'hull_radius': 0.17,
        'hull_length': 1.3333333,
        'ballast': None,
        'n_ballast': 0,
    },
}


class MassCalibration(Node):
    def __init__(self, model='submarine'):
        super().__init__('mass_calibration')
        self.model = model
        self.cfg = MODELS.get(model, MODELS['submarine'])
        self.depth = 0.0
        self.depth_vel = 0.0
        self.data_ok = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32, f'/model/{model}/pressure', self._press_cb, qos)

        self.samples = []
        self.t0 = None
        self.timer = self.create_timer(0.1, self._collect)
        self._last_print = 0

        sys.stdout.write("\n" + "="*60 + "\n")
        sys.stdout.write(f"  MASS CALIBRATION v51.0 | model={model}\n")
        sys.stdout.write("  120 сек, НЕ ТРОГАЙТЕ АППАРАТ!\n")
        sys.stdout.write("="*60 + "\n\n")
        sys.stdout.flush()

    def _press_cb(self, msg):
        new_depth = (P_Z0 - max(0.0, msg.data)) / RHO_G
        self.depth_vel = (new_depth - self.depth) / 0.02
        self.depth = new_depth
        self.data_ok = True

    def _collect(self):
        if not self.data_ok:
            return
        t = time.time()
        if self.t0 is None:
            self.t0 = t
        elapsed = t - self.t0

        self.samples.append({
            't': elapsed,
            'depth': self.depth,
            'vel_z': self.depth_vel,
        })

        # Промежуточный вывод на 60 сек
        if not hasattr(self, '_60_printed') and elapsed >= 60.0:
            self._60_printed = True
            self._print_intermediate(60.0)

        # Прогресс каждые 5 сек
        if elapsed - self._last_print >= 5.0:
            self._last_print = elapsed
            sys.stdout.write(f"\r\033[K  [{int(elapsed)}сек] Z:{self.depth:+.3f}м Vz:{self.depth_vel:+.4f}м/с")
            sys.stdout.flush()

        if elapsed >= DURATION:
            self._analyze()
            raise SystemExit

    def _print_intermediate(self, target_time):
        """Промежуточные результаты на заданном рубеже."""
        n = len(self.samples)
        if n < 10:
            return
        start_idx = n // 5
        depths = [s['depth'] for s in self.samples[start_idx:]]
        vel_zs = [s['vel_z'] for s in self.samples[start_idx:]]
        avg_vel = sum(vel_zs) / len(vel_zs)
        total_depth_change = self.samples[-1]['depth'] - self.samples[0]['depth']
        total_time = self.samples[-1]['t'] - self.samples[0]['t']
        depth_vel_avg = total_depth_change / total_time if total_time > 0 else 0
        min_vel = min(vel_zs)
        max_vel = max(vel_zs)

        sys.stdout.write("\r\033[K\n")
        sys.stdout.write("="*50 + "\n")
        sys.stdout.write(f"  ПРОМЕЖУТОЧНЫЙ РЕЗУЛЬТАТ ({int(target_time)} сек)\n")
        sys.stdout.write("="*50 + "\n\n")
        sys.stdout.write(f"  📊 СКОРОСТЬ:\n")
        sys.stdout.write(f"     Средняя Vz:      {avg_vel:+.4f} м/с\n")
        sys.stdout.write(f"     Δглубины/время:  {depth_vel_avg:+.4f} м/с\n")
        sys.stdout.write(f"     Изменение Z:     {total_depth_change:+.3f} м\n")
        sys.stdout.write(f"     Vz (мин):        {min_vel:+.4f} м/с\n")
        sys.stdout.write(f"     Vz (макс):       {max_vel:+.4f} м/с\n\n")

        # Mass calculation
        current_mass = self.cfg['mass']
        v = abs(avg_vel)
        if v > 0.001:
            zW = 15.0; zWabsW = 250.0
            F_drag = zW * v + zWabsW * v * v
            extra_mass = F_drag / 9.81
            direction = "всплывает" if avg_vel > 0 else "тонет"
            sys.stdout.write(f"  ⚖️  РАСЧЁТ:\n")
            sys.stdout.write(f"     Направление:    {direction}\n")
            sys.stdout.write(f"     F_сумма:        {F_drag:.3f} Н\n")
            if avg_vel > 0:
                sys.stdout.write(f"     ➕ Добавить:     {extra_mass:.3f} кг\n")
            else:
                sys.stdout.write(f"     ➖ Убрать:       {extra_mass:.3f} кг\n")
        else:
            sys.stdout.write(f"  ✅ Нейтральная плавучесть\n")

        sys.stdout.write("\n" + "="*50 + "\n")
        sys.stdout.write("  Ждём до 120 сек...\n")
        sys.stdout.write("="*50 + "\n\n")
        sys.stdout.flush()

    def _analyze(self):
        if len(self.samples) < 10:
            sys.stdout.write("\n  ❌ Мало данных!\n")
            return

        sys.stdout.write("\r\033[K\n")
        sys.stdout.write("\n" + "="*60 + "\n")
        sys.stdout.write("  РЕЗУЛЬТАТЫ\n")
        sys.stdout.write("="*60 + "\n\n")

        n = len(self.samples)
        start_idx = n // 5

        all_depths = [s['depth'] for s in self.samples]
        all_vel_zs = [s['vel_z'] for s in self.samples]
        depths = [s['depth'] for s in self.samples[start_idx:]]
        vel_zs = [s['vel_z'] for s in self.samples[start_idx:]]
        times = [s['t'] for s in self.samples[start_idx:]]

        avg_vel = sum(vel_zs) / len(vel_zs)
        total_depth_change = self.samples[-1]['depth'] - self.samples[0]['depth']
        total_time = self.samples[-1]['t'] - self.samples[0]['t']
        depth_vel_avg = total_depth_change / total_time if total_time > 0 else 0

        # Min/max depth
        min_depth = min(all_depths)
        max_depth = max(all_depths)

        # Velocity stats
        min_vel = min(all_vel_zs)
        max_vel = max(all_vel_zs)

        sys.stdout.write(f"  📊 ДАННЫЕ:\n")
        sys.stdout.write(f"     Образцов:        {n}\n")
        sys.stdout.write(f"     Время:           {total_time:.1f} сек\n")
        sys.stdout.write(f"     Глубина (мин):   {min_depth:+.3f} м\n")
        sys.stdout.write(f"     Глубина (макс):  {max_depth:+.3f} м\n\n")

        sys.stdout.write(f"  📊 СКОРОСТЬ:\n")
        sys.stdout.write(f"     Средняя Vz:      {avg_vel:+.4f} м/с\n")
        sys.stdout.write(f"     Δглубины/время:  {depth_vel_avg:+.4f} м/с\n")
        sys.stdout.write(f"     Изменение Z:     {total_depth_change:+.3f} м\n")
        sys.stdout.write(f"     Vz (мин):        {min_vel:+.4f} м/с\n")
        sys.stdout.write(f"     Vz (макс):       {max_vel:+.4f} м/с\n\n")

        # Acceleration
        n3 = len(vel_zs) // 3
        if n3 > 0:
            vel_start = sum(vel_zs[:n3]) / n3
            vel_end = sum(vel_zs[-n3:]) / n3
            t_start = times[n3] - times[0] if n3 > 1 else 1
            t_end = times[-1] - times[-n3] if n3 > 1 else 1

            sys.stdout.write(f"  📈 УСКОРЕНИЕ:\n")
            sys.stdout.write(f"     Vz (начало):  {vel_start:+.4f} м/с\n")
            sys.stdout.write(f"     Vz (конец):   {vel_end:+.4f} м/с\n")
            sys.stdout.write(f"     ΔVz:          {vel_end - vel_start:+.4f} м/с\n\n")

        # Mass calculation
        current_mass = self.cfg['mass']  # из SDF
        v = abs(avg_vel)

        if v > 0.001:
            # Полное сопротивление
            zW = 15.0
            zWabsW = 250.0
            F_drag = zW * v + zWabsW * v * v
            F_buoy = F_drag
            extra_mass = F_buoy / 9.81

            direction = "всплывает" if avg_vel > 0 else "тонет"

            sys.stdout.write(f"  ⚖️  РАСЧЁТ МАССЫ:\n")
            sys.stdout.write(f"     Текущая масса:     {current_mass:.2f} кг\n")
            sys.stdout.write(f"     Направление:       {direction}\n")
            sys.stdout.write(f"     F_drag (линей):    {zW * v:.3f} Н\n")
            sys.stdout.write(f"     F_drag (квадр.):   {zWabsW * v * v:.3f} Н\n")
            sys.stdout.write(f"     F_сумма:           {F_buoy:.3f} Н\n")

            if avg_vel > 0:  # всплывает
                sys.stdout.write(f"     ➕ Добавить массы:  {extra_mass:.3f} кг\n")
                sys.stdout.write(f"     Новая масса:       {current_mass + extra_mass:.3f} кг\n")
            else:  # тонет
                sys.stdout.write(f"     ➖ Убрать массы:    {extra_mass:.3f} кг\n")
                sys.stdout.write(f"     Новая масса:       {current_mass - extra_mass:.3f} кг\n")
        else:
            sys.stdout.write(f"  ✅ Аппарат НЕ всплывает (Vz ≈ 0)\n")

        # Buoyancy info
        c = self.cfg
        hull_vol = math.pi * c['hull_radius']**2 * c['hull_length']
        if c['ballast']:
            bx, by, bz = c['ballast']
            ballast_vol = c['n_ballast'] * bx * by * bz
        else:
            ballast_vol = 0.0
        total_vol = hull_vol + ballast_vol
        buoy = 1000 * 9.81 * total_vol
        weight = current_mass * 9.81

        sys.stdout.write(f"\n  🌊 ПЛАВУЧЕСТЬ (расчётная):\n")
        sys.stdout.write(f"     Объём корпуса:     {hull_vol:.4f} м³\n")
        sys.stdout.write(f"     Объём балластов:   {ballast_vol:.4f} м³\n")
        sys.stdout.write(f"     Объём суммарный:   {total_vol:.4f} м³\n")
        sys.stdout.write(f"     Плавучесть:        {buoy:.1f} Н\n")
        sys.stdout.write(f"     Вес:               {weight:.1f} Н\n")
        sys.stdout.write(f"     Баланс (расчёт):   {buoy - weight:+.1f} Н\n")

        sys.stdout.write("\n" + "="*60 + "\n")
        sys.stdout.flush()


def main():
    print("=" * 40)
    print("  MASS CALIBRATION")
    print("=" * 40)
    print("  1: submarine (с балластами)")
    print("  2: submarine_nb (без балластов)")
    r = input("  Выбор [1]: ").strip()
    model = 'submarine_nb' if r == '2' else 'submarine'
    print(f"  -> Модель: {model}\n")

    rclpy.init()
    node = MassCalibration(model)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
