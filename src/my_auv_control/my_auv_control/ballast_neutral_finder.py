#!/usr/bin/env python3
"""Ballast Neutral Finder (v3) — поиск нейтральной плавучести по УСКОРЕНИЮ.

Почему ускорение, а не скорость:
  Чистая вертикальная сила: F = плавучесть - вес - сопротивление(v).
  При НЕЙТРАЛИ F=0 -> ускорение a = F/m = 0. А вот скорость на установившемся
  режиме (терминальная) НЕ ноль даже близко к нейтрали + её искажает инерция.
  Поэтому метрика нейтрали = ускорение a = dVz/dt (вторая производная глубины).
    a > 0  -> всплывает (избыток плавучести)
    a < 0  -> тонет (недостаток)
    a ~ 0  -> НЕЙТРАЛЬ

Алгоритм: плавный свип, шаг падает ПОСЛЕ перерегулирования (овершута).
  Граница объёма (0 или 1) ИЗМЕРЯЕТСЯ; «недостижимо» объявляется только если
  НА границе знак всё ещё неправильный.

Vz и a считаются по БАРОМЕТРУ (глубина -> dVz/dt). НЕ управляйте аппаратом!
"""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MODEL = 'submarine'
MAX_BALLAST_VOL = 0.015     # м³ на бак (из SDF max_volume)
P_Z0 = 101325.0
RHO_G = 9810.0


class BallastNeutralFinder(Node):
    def __init__(self):
        super().__init__('ballast_neutral_finder')

        self.declare_parameter('settle_time', 4.0)   # успокоение/наполнение пузыря
        self.declare_parameter('measure_time', 4.0)  # окно измерения ускорения
        self.declare_parameter('acc_tol', 0.005)     # м/с² — порог нейтрали по УСКОРЕНИЮ
        self.declare_parameter('step0', 0.25)
        self.declare_parameter('step_min', 0.002)
        self.declare_parameter('start_vol', 0.5)
        self.declare_parameter('max_iter', 40)
        self.settle_time  = self.get_parameter('settle_time').value
        self.measure_time = self.get_parameter('measure_time').value
        self.acc_tol      = self.get_parameter('acc_tol').value
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
        self.vz = 0.0           # вертикальная скорость (вверх +)
        self.prev_vz = 0.0
        self.acc = 0.0          # вертикальное ускорение (вверх +)
        self.data_ok = False

        self.prev_sign = None
        self.iter = 0
        self.phase = 'INIT'
        self.t_phase = None
        self.acc_acc = 0.0
        self.vz_acc = 0.0
        self.n = 0
        self.best = None
        self.hit_top = False
        self.hit_bottom = False

        self._set_volume(self.vol)
        self.timer = self.create_timer(0.1, self._loop)

        sys.stdout.write("\n" + "=" * 64 + "\n")
        sys.stdout.write("  ПОИСК НЕЙТРАЛИ ПО УСКОРЕНИЮ (плавный свип, шаг падает на овершутах)\n")
        sys.stdout.write(f"  step0={self.step} settle={self.settle_time}s "
                         f"measure={self.measure_time}s acc_tol={self.acc_tol} м/с²\n")
        sys.stdout.write("  Метрика = ускорение a (a~0 => нейтраль). НЕ управляйте аппаратом!\n")
        sys.stdout.write("=" * 64 + "\n\n")
        sys.stdout.flush()

    def _press(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        self.depth = (P_Z0 - msg.data) / RHO_G
        if self.prev_depth is not None and self.prev_t is not None:
            dt = t - self.prev_t
            if dt > 1e-3:
                raw_vz = (self.depth - self.prev_depth) / dt          # вверх +
                vz_f = 0.7 * self.vz + 0.3 * raw_vz
                raw_acc = (vz_f - self.vz) / dt
                self.acc = 0.8 * self.acc + 0.2 * raw_acc
                self.vz = vz_f
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
            self.phase = 'SETTLE'; self.t_phase = t
            return

        if self.phase == 'SETTLE':
            if t - self.t_phase >= self.settle_time:
                self.phase = 'MEASURE'; self.t_phase = t
                self.acc_acc = 0.0; self.vz_acc = 0.0; self.n = 0
            else:
                sys.stdout.write(f"\r[итер {self.iter}] vol={self.vol:.4f} step={self.step:.4f} "
                                 f"наполнение... a={self.acc:+.4f} Vz={self.vz:+.3f}   ")
                sys.stdout.flush()
            return

        if self.phase == 'MEASURE':
            self.acc_acc += self.acc; self.vz_acc += self.vz; self.n += 1
            if t - self.t_phase >= self.measure_time:
                a = self.acc_acc / max(1, self.n)
                vz = self.vz_acc / max(1, self.n)
                self._evaluate(a, vz)
            else:
                sys.stdout.write(f"\r[итер {self.iter}] vol={self.vol:.4f} step={self.step:.4f} "
                                 f"измерение a={self.acc:+.4f} Vz={self.vz:+.3f}   ")
                sys.stdout.flush()
            return

    def _evaluate(self, a, vz):
        sign = 1 if a > self.acc_tol else -1 if a < -self.acc_tol else 0
        d = "ВСПЛЫВАЕТ" if sign > 0 else "ТОНЕТ" if sign < 0 else "НЕЙТРАЛЬ"
        sys.stdout.write(f"\r\033[K[итер {self.iter}] vol={self.vol:.4f} step={self.step:.4f}  "
                         f"a_ср={a:+.4f} м/с²  Vz_ср={vz:+.3f}  -> {d}\n")
        sys.stdout.flush()

        if self.best is None or abs(a) < abs(self.best[1]):
            self.best = (self.vol, a, vz)

        if sign == 0:
            self._finish("ускорение в допуске (нейтраль)"); return

        # на границе и знак неправильный -> действительно недостижимо
        if (self.vol >= 0.999 and sign < 0) or (self.vol <= 0.001 and sign > 0):
            self._finish("НЕЙТРАЛЬ НЕДОСТИЖИМА (даже на границе объёма знак не тот)"); return

        self.iter += 1
        if self.iter >= self.max_iter:
            self._finish("исчерпан лимит итераций"); return

        # направление: всплывает (a>0) -> меньше объёма; тонет (a<0) -> больше
        want = -1 if sign > 0 else +1

        # овершут -> уменьшаем шаг (как просил пользователь: позже, после перерегулирования)
        if self.prev_sign is not None and sign != self.prev_sign:
            self.step *= 0.5
            sys.stdout.write(f"    \u21b3 перерегулирование: шаг -> {self.step:.4f}\n")
            sys.stdout.flush()
        self.prev_sign = sign

        if self.step < self.step_min:
            self._finish("шаг меньше порога (сошлось)"); return

        new_vol = max(0.0, min(1.0, self.vol + want * self.step))
        self._set_volume(new_vol)
        t = self.get_clock().now().nanoseconds / 1e9
        self.phase = 'SETTLE'; self.t_phase = t

    def _finish(self, reason):
        self.phase = 'DONE'
        vol, a, vz = self.best
        each = vol * MAX_BALLAST_VOL
        sys.stdout.write("\n" + "=" * 64 + "\n  РЕЗУЛЬТАТ (" + reason + ")\n" + "=" * 64 + "\n")
        sys.stdout.write(f"  Нейтральный объём (норм. 0..1):   {vol:.4f}\n")
        sys.stdout.write(f"  Объём на 1 бак:                   {each*1000:.3f} л\n")
        sys.stdout.write(f"  Объём на 4 бака:                  {4*each*1000:.3f} л\n")
        sys.stdout.write(f"  Остаточное ускорение:             {a:+.4f} м/с²\n")
        sys.stdout.write(f"  Остаточная скорость:              {vz:+.4f} м/с\n")
        if 'НЕДОСТИЖИМА' in reason:
            sys.stdout.write("\n  ⚠ Балласт не может уравновесить аппарат в диапазоне баков.\n"
                             "    Менять массу / max_volume / neutral_volume.\n")
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
