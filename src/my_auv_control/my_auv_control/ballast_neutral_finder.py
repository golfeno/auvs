#!/usr/bin/env python3
"""Ballast Neutral Finder (v2) — поиск нейтральной плавучести плавным свипом.

Алгоритм (по просьбе: шаг уменьшать ПОСЛЕ перерегулирования, а не сразу):
  RELAY / HILL-CLIMB с уменьшением шага на овершутах.
  - Идём в одну сторону по объёму с шагом `step`, ПОКА знак Vz не сменится
    (т.е. пока не перерегулируем — не проскочим нейтраль).
  - Как только Vz сменил знак (овершут) → разворачиваемся И уменьшаем шаг вдвое.
  - Так амплитуда колебаний вокруг нейтрали затухает → сходимся к Vz≈0.

Vz измеряется по БАРОМЕТРУ (производная глубины) — это однозначно вертикальная
скорость в мире. Одометрийная twist.linear.z тут НЕ годится: корпус повёрнут
на 90°, и её z — это не мировая вертикаль.

Запуск:
  ros2 run my_auv_control ballast_neutral
Параметры:
  step0 (нач. шаг объёма), step_min (порог остановки по шагу),
  settle_time, measure_time, vz_tol.
"""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MODEL = 'submarine'
MAX_BALLAST_VOL = 0.015
P_Z0 = 101325.0
RHO_G = 9810.0


class BallastNeutralFinder(Node):
    def __init__(self):
        super().__init__('ballast_neutral_finder')

        self.declare_parameter('settle_time', 6.0)
        self.declare_parameter('measure_time', 4.0)
        self.declare_parameter('vz_tol', 0.01)
        self.declare_parameter('step0', 0.25)     # начальный шаг объёма
        self.declare_parameter('step_min', 0.002) # остановка, когда шаг стал мал
        self.declare_parameter('start_vol', 0.5)
        self.declare_parameter('max_iter', 40)
        self.settle_time  = self.get_parameter('settle_time').value
        self.measure_time = self.get_parameter('measure_time').value
        self.vz_tol       = self.get_parameter('vz_tol').value
        self.step         = self.get_parameter('step0').value
        self.step_min     = self.get_parameter('step_min').value
        self.vol          = self.get_parameter('start_vol').value
        self.max_iter     = self.get_parameter('max_iter').value

        self.pub_b = [self.create_publisher(
            Float64, f'/model/{MODEL}/ballast_{i}/volume', 10) for i in range(1, 5)]

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32, f'/model/{MODEL}/pressure', self._press, qos)

        self.depth = 0.0
        self.prev_depth = None
        self.prev_t = None
        self.vz = 0.0           # производная глубины (вверх +)
        self.data_ok = False

        # состояние свипа
        self.dir = None         # +1 (увеличиваем объём) или -1 (уменьшаем)
        self.prev_sign = None   # знак Vz на прошлом измерении
        self.iter = 0
        self.phase = 'INIT'
        self.t_phase = None
        self.vz_acc = 0.0
        self.vz_n = 0
        self.best = None
        # диагностика недостижимости
        self.hit_top = False
        self.hit_bottom = False

        self._set_volume(self.vol)
        self.timer = self.create_timer(0.1, self._loop)

        sys.stdout.write("\n" + "=" * 64 + "\n")
        sys.stdout.write("  ПОИСК НЕЙТРАЛЬНОЙ ПЛАВУЧЕСТИ (плавный свип, шаг падает на овершутах)\n")
        sys.stdout.write(f"  step0={self.step} settle={self.settle_time}s "
                         f"measure={self.measure_time}s tol={self.vz_tol} м/с\n")
        sys.stdout.write("  Vz считается по БАРОМЕТРУ. НЕ управляйте аппаратом!\n")
        sys.stdout.write("=" * 64 + "\n\n")
        sys.stdout.flush()

    def _press(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        self.depth = (P_Z0 - msg.data) / RHO_G
        if self.prev_depth is not None and self.prev_t is not None:
            dt = t - self.prev_t
            if dt > 1e-3:
                raw = (self.depth - self.prev_depth) / dt   # вверх +
                self.vz = 0.7 * self.vz + 0.3 * raw
        self.prev_depth = self.depth
        self.prev_t = t
        self.data_ok = True

    def _set_volume(self, norm):
        norm = max(0.0, min(1.0, norm))
        self.vol = norm
        for p in self.pub_b:
            p.publish(Float64(data=norm * MAX_BALLAST_VOL))

    def _loop(self):
        if not self.data_ok:
            return
        t = self.get_clock().now().nanoseconds / 1e9

        if self.phase == 'INIT':
            self._start_settle(t)
            return

        if self.phase == 'SETTLE':
            if t - self.t_phase >= self.settle_time:
                self.phase = 'MEASURE'; self.t_phase = t
                self.vz_acc = 0.0; self.vz_n = 0
            else:
                sys.stdout.write(f"\r[итер {self.iter}] vol={self.vol:.4f} step={self.step:.4f} "
                                 f"успокоение Vz={self.vz:+.4f}   ")
                sys.stdout.flush()
            return

        if self.phase == 'MEASURE':
            self.vz_acc += self.vz; self.vz_n += 1
            if t - self.t_phase >= self.measure_time:
                self._evaluate(self.vz_acc / max(1, self.vz_n))
            else:
                sys.stdout.write(f"\r[итер {self.iter}] vol={self.vol:.4f} step={self.step:.4f} "
                                 f"измерение Vz={self.vz:+.4f}   ")
                sys.stdout.flush()
            return

    def _start_settle(self, t):
        self.phase = 'SETTLE'; self.t_phase = t

    def _evaluate(self, vz_avg):
        sign = 1 if vz_avg > self.vz_tol else -1 if vz_avg < -self.vz_tol else 0
        d = "ВСПЛЫВАЕТ" if sign > 0 else "ТОНЕТ" if sign < 0 else "НЕЙТРАЛЬ"
        sys.stdout.write(f"\r\033[K[итер {self.iter}] vol={self.vol:.4f} step={self.step:.4f} "
                         f"Vz_ср={vz_avg:+.4f} -> {d}\n")
        sys.stdout.flush()

        if self.best is None or abs(vz_avg) < abs(self.best[1]):
            self.best = (self.vol, vz_avg)

        # достигли нейтрали
        if sign == 0:
            self._finish("Vz в допуске"); return

        self.iter += 1
        if self.iter >= self.max_iter:
            self._finish("исчерпан лимит итераций"); return

        # направление движения объёма:
        # тонет (sign<0) → мало плавучести → НАДО БОЛЬШЕ объёма → dir=+1
        # всплывает (sign>0) → много → МЕНЬШЕ объёма → dir=-1
        want_dir = +1 if sign < 0 else -1

        # овершут = знак Vz сменился относительно прошлого измерения →
        # значит проскочили нейтраль → УМЕНЬШАЕМ ШАГ (как просил пользователь)
        if self.prev_sign is not None and sign != self.prev_sign:
            self.step *= 0.5
            sys.stdout.write(f"    ↳ перерегулирование: шаг уменьшен -> {self.step:.4f}\n")
            sys.stdout.flush()
        self.prev_sign = sign

        if self.step < self.step_min:
            self._finish("шаг стал меньше порога"); return

        # двигаем объём
        new_vol = self.vol + want_dir * self.step
        # детект упора в границы (диагностика недостижимости)
        if new_vol >= 1.0:
            new_vol = 1.0; self.hit_top = True
        if new_vol <= 0.0:
            new_vol = 0.0; self.hit_bottom = True
        # если упёрлись в границу и всё равно та же сторона — нейтраль недостижима
        if (self.hit_top and sign < 0) or (self.hit_bottom and sign > 0):
            self._finish("НЕЙТРАЛЬ НЕДОСТИЖИМА (упор в границу объёма)"); return

        self._set_volume(new_vol)
        t = self.get_clock().now().nanoseconds / 1e9
        self._start_settle(t)

    def _finish(self, reason):
        self.phase = 'DONE'
        vol, vz = self.best
        each = vol * MAX_BALLAST_VOL
        sys.stdout.write("\n" + "=" * 64 + "\n  РЕЗУЛЬТАТ (" + reason + ")\n" + "=" * 64 + "\n")
        sys.stdout.write(f"  Нейтральный объём (норм. 0..1):   {vol:.4f}\n")
        sys.stdout.write(f"  Объём на 1 бак:                   {each*1000:.3f} л\n")
        sys.stdout.write(f"  Объём на 4 бака:                  {4*each*1000:.3f} л\n")
        sys.stdout.write(f"  Остаточная Vz:                    {vz:+.4f} м/с\n")
        if 'НЕДОСТИЖИМА' in reason:
            sys.stdout.write("\n  ⚠ Балласт НЕ может уравновесить аппарат в этом диапазоне.\n"
                             "    Нужно менять массу/вытеснение/плотность воды.\n"
                             "    Если ТОНЕТ при полных баках — аппарат слишком тяжёлый\n"
                             "    или BuoyancyEngine не отрабатывает (проверь мост ballast_*/volume).\n")
        else:
            sys.stdout.write(f"\n  -> впиши в models.py:  bz_neutral = {vol:.3f}\n")
        sys.stdout.write("=" * 64 + "\n")
        sys.stdout.flush()
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
