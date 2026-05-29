#!/usr/bin/env python3
"""Ballast Neutral Finder (v1) — поиск нейтральной плавучести.

Идея: плавно меняем НОРМИРОВАННЫЙ объём балласта (0..1, одинаково на все 4 бака)
и измеряем вертикальную скорость Vz. Нейтраль = объём, при котором Vz ≈ 0
(аппарат не всплывает и не тонет).

Алгоритм: МЕТОД БИСЕКЦИИ (деление пополам).
  - При объёме vol измеряем установившуюся Vz (усредняя за окно).
  - Vz > 0 (всплывает) → слишком много плавучести → уменьшаем объём.
  - Vz < 0 (тонет)     → мало плавучести → увеличиваем объём.
  Границы [lo, hi] сходятся к нейтрали.

Перед измерением на каждом шаге даём аппарату «успокоиться» (settling time),
чтобы Vz вышла на установившееся значение.

Запуск:
  ros2 run my_auv_control ballast_neutral_finder
Требует: запущенную симуляцию (gz + bridge + fake_barometer),
         мост топиков ballast_*/volume и /model/submarine/odometry.
"""
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MODEL = 'submarine'
MAX_BALLAST_VOL = 0.003     # м³ на бак (из SDF max_volume)
P_Z0 = 101325.0
RHO_G = 9810.0


class BallastNeutralFinder(Node):
    def __init__(self):
        super().__init__('ballast_neutral_finder')

        # --- параметры поиска ---
        self.declare_parameter('settle_time', 6.0)   # сек на успокоение после смены объёма
        self.declare_parameter('measure_time', 4.0)  # сек измерения Vz
        self.declare_parameter('vz_tol', 0.01)       # м/с — порог «нейтрали»
        self.declare_parameter('max_iter', 18)       # макс. итераций бисекции
        self.settle_time  = self.get_parameter('settle_time').value
        self.measure_time = self.get_parameter('measure_time').value
        self.vz_tol       = self.get_parameter('vz_tol').value
        self.max_iter     = self.get_parameter('max_iter').value

        # --- публикаторы балласта (4 бака) ---
        self.pub_b = [self.create_publisher(
            Float64, f'/model/{MODEL}/ballast_{i}/volume', 10) for i in range(1, 5)]

        # --- подписки (Vz из одометрии + глубина из барометра) ---
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, f'/model/{MODEL}/odometry', self._odom, 10)
        self.create_subscription(Float32, f'/model/{MODEL}/pressure', self._press, qos)

        self.vz = 0.0
        self.depth = 0.0
        self.prev_depth = None
        self.data_ok = False

        # --- состояние машины поиска ---
        self.lo = 0.0          # объём, при котором ТОНЕТ (Vz<0)
        self.hi = 1.0          # объём, при котором ВСПЛЫВАЕТ (Vz>0)
        self.vol = 0.5         # текущий пробный объём
        self.iter = 0
        self.phase = 'INIT'    # INIT -> SETTLE -> MEASURE -> (next) -> DONE
        self.t_phase = None
        self.vz_acc = 0.0
        self.vz_n = 0
        self.best = None

        self._set_volume(self.vol)
        self.timer = self.create_timer(0.1, self._loop)

        sys.stdout.write("\n" + "=" * 64 + "\n")
        sys.stdout.write("  ПОИСК НЕЙТРАЛЬНОЙ ПЛАВУЧЕСТИ (бисекция по объёму балласта)\n")
        sys.stdout.write(f"  settle={self.settle_time}s measure={self.measure_time}s "
                         f"tol={self.vz_tol} м/с\n")
        sys.stdout.write("  НЕ управляйте аппаратом во время теста!\n")
        sys.stdout.write("=" * 64 + "\n\n")
        sys.stdout.flush()

    # ---------- сенсоры ----------
    def _press(self, msg):
        self.depth = (P_Z0 - msg.data) / RHO_G

    def _odom(self, msg):
        # Vz прямо из одометрии (вверх +)
        self.vz = msg.twist.twist.linear.z
        self.data_ok = True

    # ---------- управление балластом ----------
    def _set_volume(self, norm):
        norm = max(0.0, min(1.0, norm))
        for p in self.pub_b:
            p.publish(Float64(data=norm * MAX_BALLAST_VOL))

    # ---------- машина поиска ----------
    def _loop(self):
        if not self.data_ok:
            return
        t = self.get_clock().now().nanoseconds / 1e9

        if self.phase == 'INIT':
            self._start_settle(t)
            return

        if self.phase == 'SETTLE':
            if t - self.t_phase >= self.settle_time:
                self.phase = 'MEASURE'
                self.t_phase = t
                self.vz_acc = 0.0
                self.vz_n = 0
            else:
                sys.stdout.write(
                    f"\r[итер {self.iter}] vol={self.vol:.4f} "
                    f"успокоение... Vz={self.vz:+.4f} м/с   ")
                sys.stdout.flush()
            return

        if self.phase == 'MEASURE':
            self.vz_acc += self.vz
            self.vz_n += 1
            if t - self.t_phase >= self.measure_time:
                vz_avg = self.vz_acc / max(1, self.vz_n)
                self._evaluate(vz_avg)
            else:
                sys.stdout.write(
                    f"\r[итер {self.iter}] vol={self.vol:.4f} "
                    f"измерение... Vz={self.vz:+.4f} м/с   ")
                sys.stdout.flush()
            return

    def _start_settle(self, t):
        self._set_volume(self.vol)
        self.phase = 'SETTLE'
        self.t_phase = t

    def _evaluate(self, vz_avg):
        direction = ("ВСПЛЫВАЕТ" if vz_avg > self.vz_tol else
                     "ТОНЕТ" if vz_avg < -self.vz_tol else "НЕЙТРАЛЬ")
        sys.stdout.write(
            f"\r\033[K[итер {self.iter}] vol={self.vol:.4f}  "
            f"Vz_ср={vz_avg:+.4f} м/с  -> {direction}\n")
        sys.stdout.flush()

        # лучший результат (минимум |Vz|)
        if self.best is None or abs(vz_avg) < abs(self.best[1]):
            self.best = (self.vol, vz_avg)

        # критерии остановки
        self.iter += 1
        if abs(vz_avg) <= self.vz_tol or self.iter >= self.max_iter:
            self._finish()
            return

        # бисекция
        if vz_avg > 0:        # всплывает -> много плавучести -> уменьшить объём
            self.hi = self.vol
        else:                 # тонет -> мало плавучести -> увеличить объём
            self.lo = self.vol
        self.vol = 0.5 * (self.lo + self.hi)

        # следующий цикл
        t = self.get_clock().now().nanoseconds / 1e9
        self._start_settle(t)

    def _finish(self):
        self.phase = 'DONE'
        vol, vz = self.best
        # переводим в физические величины
        vol_m3_each = vol * MAX_BALLAST_VOL
        vol_m3_total = 4 * vol_m3_each
        sys.stdout.write("\n" + "=" * 64 + "\n")
        sys.stdout.write("  РЕЗУЛЬТАТ\n")
        sys.stdout.write("=" * 64 + "\n")
        sys.stdout.write(f"  Нейтральный объём (норм. 0..1):   {vol:.4f}\n")
        sys.stdout.write(f"  Объём на 1 бак:                   {vol_m3_each*1000:.3f} л "
                         f"({vol_m3_each:.6f} м³)\n")
        sys.stdout.write(f"  Объём на 4 бака:                  {vol_m3_total*1000:.3f} л\n")
        sys.stdout.write(f"  Остаточная Vz:                    {vz:+.4f} м/с\n")
        sys.stdout.write(f"\n  -> впиши в models.py:  bz_neutral = {vol:.3f}\n")
        sys.stdout.write("=" * 64 + "\n")
        sys.stdout.flush()
        # удерживаем найденный нейтральный объём
        self._set_volume(vol)
        raise SystemExit


def main():
    rclpy.init()
    node = BallastNeutralFinder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
