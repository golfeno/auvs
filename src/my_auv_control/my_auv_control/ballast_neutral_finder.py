#!/usr/bin/env python3
"""Ballast Calibration (v5) — нейтральная плавучесть + регулировка ДИФФЕРЕНТА.

СТАДИЯ 1 — ГЛУБИНА (нейтральная плавучесть):
  Метрика = установившаяся (терминальная) скорость Vz по барометру.
  Истинная нейтраль = Vz_терм -> 0. Плавный свип (шаг падает на овершутах),
  затем ЛИНЕЙНАЯ ИНТЕРПОЛЯЦИЯ корня между двумя точками, охватывающими ноль
  (Vz<0 и Vz>0) -> точность лучше шага, плюс проверочное измерение.

СТАДИЯ 2 — ДИФФЕРЕНТ (баланс нос/корма):
  При найденном базовом объёме перераспределяем объём между носовыми баками
  (ballast_1,2) и кормовыми (ballast_3,4), СОХРАНЯЯ суммарный объём:
        нос  = base + trim   (2 бака)
        корма= base - trim   (2 бака)   -> суммарный объём неизменен.
  Метрика дифферента = УГОЛ тангажа pitch (рад) из одометрии (asin(2(wy-zx))).
  При горизонте pitch~0. Hill-climb минимизирует |pitch| -> trim к балансу.
  (Если корма/нос уже сбалансированы, узел быстро сойдётся к trim=0.)

Vz по барометру, ориентация по одометрии. НЕ управляйте аппаратом!
"""
import sys, math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MODEL = 'submarine'
MAX_BALLAST_VOL = 0.015
P_Z0 = 101325.0
RHO_G = 9810.0
# Карта баков: индексы 0..3 -> ballast_1..4. Нос = 1,2 (X>0); корма = 3,4 (X<0).
BOW = (0, 1)
STERN = (2, 3)


class BallastCalib(Node):
    def __init__(self):
        super().__init__('ballast_calibration')

        self.declare_parameter('settle_time', 30.0)  # медленный насос (0.001 м3/с): бак 15л за 15с + терминал
        self.declare_parameter('measure_time', 6.0)
        self.declare_parameter('vz_tol', 0.01)
        self.declare_parameter('acc_max', 0.01)
        self.declare_parameter('step0', 0.25)
        self.declare_parameter('step_min', 0.004)
        self.declare_parameter('start_vol', 0.5)
        self.declare_parameter('trim_step0', 0.1)
        self.declare_parameter('trim_step_min', 0.005)
        self.declare_parameter('pitch_tol', 0.017)  # ~1° допуск по тангажу
        self.declare_parameter('max_iter', 40)
        self.settle_time  = self.get_parameter('settle_time').value
        self.measure_time = self.get_parameter('measure_time').value
        self.vz_tol       = self.get_parameter('vz_tol').value
        self.acc_max      = self.get_parameter('acc_max').value
        self.step         = self.get_parameter('step0').value
        self.step_min     = self.get_parameter('step_min').value
        self.vol          = self.get_parameter('start_vol').value
        self.trim_step    = self.get_parameter('trim_step0').value
        self.trim_step_min= self.get_parameter('trim_step_min').value
        self.pitch_tol    = self.get_parameter('pitch_tol').value
        self.max_iter     = self.get_parameter('max_iter').value

        self.pub_b = [self.create_publisher(
            Float64, f'/model/sub_ballast_{i}/buoyancy_engine', 10) for i in range(1, 5)]

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32, f'/model/{MODEL}/pressure', self._press, qos)
        self.create_subscription(Odometry, f'/model/{MODEL}/odometry', self._odom, 10)

        # сенсоры
        self.depth = 0.0; self.prev_depth = None; self.prev_t = None
        self.vz = 0.0; self.acc = 0.0
        self.pitch_metric = 0.0
        self.data_ok = False

        # стадия
        self.stage = 'DEPTH'        # DEPTH -> TRIM -> DONE
        self.phase = 'INIT'
        self.t_phase = None
        self.acc_acc = self.vz_acc = self.pm_acc = 0.0
        self.n = 0

        # стадия глубины
        self.base = self.vol
        self.prev_sign = None
        self.lo = None; self.hi = None      # bracket (vol, vz)
        self.iter = 0
        self.best = None

        # стадия дифферента
        self.trim = 0.0
        self.prev_trim_cost = None
        self.trim_dir = +1
        self.trim_iter = 0
        self.best_trim = None

        self._set_depth(self.vol)
        self.timer = self.create_timer(0.1, self._loop)

        sys.stdout.write("\n" + "=" * 66 + "\n")
        sys.stdout.write("  КАЛИБРОВКА БАЛЛАСТА: 1) нейтраль по Vz  2) дифферент по наклону\n")
        sys.stdout.write(f"  settle={self.settle_time}s measure={self.measure_time}s\n")
        sys.stdout.write("  НЕ управляйте аппаратом!\n")
        sys.stdout.write("=" * 66 + "\n\n")
        sys.stdout.flush()

    # ---------- сенсоры ----------
    def _press(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        self.depth = (P_Z0 - msg.data) / RHO_G
        if self.prev_depth is not None and self.prev_t is not None:
            dt = t - self.prev_t
            if dt > 1e-3:
                raw = (self.depth - self.prev_depth) / dt
                vzf = 0.7 * self.vz + 0.3 * raw
                self.acc = 0.8 * self.acc + 0.2 * (vzf - self.vz) / dt
                self.vz = vzf
        self.prev_depth = self.depth; self.prev_t = t; self.data_ok = True

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        # УГОЛ тангажа (рад). При горизонте ~0; нос вверх/вниз -> +/-.
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch_metric = math.asin(sinp)

    # ---------- управление баками ----------
    def _set_depth(self, norm):
        """Одинаковый объём на все баки (норм. 0..1)."""
        norm = max(0.0, min(1.0, norm)); self.vol = norm
        for p in self.pub_b:
            p.publish(Float64(data=norm * MAX_BALLAST_VOL))

    def _set_trim(self, base, trim):
        """Нос = base+trim, корма = base-trim (суммарный объём сохраняется)."""
        bow = max(0.0, min(1.0, base + trim))
        stern = max(0.0, min(1.0, base - trim))
        for i in BOW:
            self.pub_b[i].publish(Float64(data=bow * MAX_BALLAST_VOL))
        for i in STERN:
            self.pub_b[i].publish(Float64(data=stern * MAX_BALLAST_VOL))

    # ---------- цикл ----------
    def _loop(self):
        if not self.data_ok:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        if self.phase == 'INIT':
            self.phase = 'SETTLE'; self.t_phase = t; return
        if self.phase == 'SETTLE':
            if t - self.t_phase >= self.settle_time:
                self.phase = 'MEASURE'; self.t_phase = t
                self.acc_acc = self.vz_acc = self.pm_acc = 0.0; self.n = 0
            else:
                self._progress("успокоение")
            return
        if self.phase == 'MEASURE':
            self.acc_acc += self.acc; self.vz_acc += self.vz
            self.pm_acc += self.pitch_metric; self.n += 1
            if t - self.t_phase >= self.measure_time:
                a = self.acc_acc / self.n; vz = self.vz_acc / self.n; pm = self.pm_acc / self.n
                if self.stage == 'DEPTH':
                    self._eval_depth(a, vz)
                else:
                    self._eval_trim(pm)
            else:
                self._progress("измерение")
            return

    def _progress(self, what):
        if self.stage == 'DEPTH':
            sys.stdout.write(f"\r[Г{self.iter}] vol={self.vol:.4f} step={self.step:.4f} "
                             f"{what} Vz={self.vz:+.3f} a={self.acc:+.4f}   ")
        else:
            sys.stdout.write(f"\r[Д{self.trim_iter}] trim={self.trim:+.4f} step={self.trim_step:.4f} "
                             f"{what} тангаж={math.degrees(self.pitch_metric):+.2f}°   ")
        sys.stdout.flush()

    # ---------- СТАДИЯ 1: ГЛУБИНА ----------
    def _eval_depth(self, a, vz):
        terminal = abs(a) < self.acc_max
        sign = 1 if vz > self.vz_tol else -1 if vz < -self.vz_tol else 0
        d = "ВСПЛЫВ" if sign > 0 else "ТОНЕТ" if sign < 0 else "НЕЙТР"
        warn = "" if terminal else "  [не терминал]"
        sys.stdout.write(f"\r\033[K[Г{self.iter}] vol={self.vol:.4f} step={self.step:.4f}  "
                         f"Vz={vz:+.4f}  a={a:+.4f}  -> {d}{warn}\n"); sys.stdout.flush()

        if self.best is None or abs(vz) < abs(self.best[1]):
            self.best = (self.vol, vz)
        # обновляем bracket для интерполяции
        if vz > 0:
            self.hi = (self.vol, vz)
        elif vz < 0:
            self.lo = (self.vol, vz)

        # сошлись по допуску
        if sign == 0 and terminal:
            self._finish_depth(self.vol, "Vz в допуске"); return
        # если есть полный bracket и шаг мал -> интерполяция корня
        if self.lo and self.hi and self.step <= self.step_min:
            lo_v, lo_vz = self.lo; hi_v, hi_vz = self.hi
            root = lo_v + (0 - lo_vz) * (hi_v - lo_v) / (hi_vz - lo_vz)
            self._finish_depth(root, "интерполяция корня"); return

        self.iter += 1
        if self.iter >= self.max_iter:
            self._finish_depth(self.best[0], "лимит итераций"); return

        if (self.vol >= 0.999 and sign < 0) or (self.vol <= 0.001 and sign > 0):
            self._finish_depth(self.vol, "упор в границу"); return

        want = -1 if sign > 0 else +1
        if self.prev_sign is not None and sign != self.prev_sign:
            self.step *= 0.5
        self.prev_sign = sign
        self._set_depth(max(0.0, min(1.0, self.vol + want * self.step)))
        self.phase = 'SETTLE'; self.t_phase = self.get_clock().now().nanoseconds / 1e9

    def _finish_depth(self, neutral, reason):
        self.base = max(0.0, min(1.0, neutral))
        sys.stdout.write("\n" + "-" * 66 + "\n")
        sys.stdout.write(f"  СТАДИЯ 1 (ГЛУБИНА) готова [{reason}]: "
                         f"bz_neutral = {self.base:.4f}\n")
        sys.stdout.write(f"  Переход к СТАДИИ 2 — регулировка дифферента...\n")
        sys.stdout.write("-" * 66 + "\n"); sys.stdout.flush()
        # старт стадии дифферента
        self.stage = 'TRIM'
        self.trim = 0.0
        self._set_trim(self.base, self.trim)
        self.phase = 'SETTLE'; self.t_phase = self.get_clock().now().nanoseconds / 1e9

    # ---------- СТАДИЯ 2: ДИФФЕРЕНТ ----------
    def _eval_trim(self, pm):
        cost = abs(pm)   # pm — угол тангажа (рад)
        sys.stdout.write(f"\r\033[K[Д{self.trim_iter}] trim={self.trim:+.4f} step={self.trim_step:.4f}  "
                         f"тангаж={math.degrees(pm):+.2f}°  |{math.degrees(cost):.2f}°|\n"); sys.stdout.flush()

        if self.best_trim is None or cost < self.best_trim[1]:
            self.best_trim = (self.trim, cost)

        if cost < self.pitch_tol:
            self._finish_trim("наклон в допуске"); return

        self.trim_iter += 1
        if self.trim_iter >= self.max_iter:
            self._finish_trim("лимит итераций"); return

        # hill-climb по |наклон|: если стало хуже — развернуться и уменьшить шаг
        if self.prev_trim_cost is not None and cost > self.prev_trim_cost:
            self.trim_dir *= -1
            self.trim_step *= 0.5
        self.prev_trim_cost = cost

        if self.trim_step < self.trim_step_min:
            self._finish_trim("шаг дифферента мал (сошлось)"); return

        self.trim = max(-0.5, min(0.5, self.trim + self.trim_dir * self.trim_step))
        self._set_trim(self.base, self.trim)
        self.phase = 'SETTLE'; self.t_phase = self.get_clock().now().nanoseconds / 1e9

    def _finish_trim(self, reason):
        self.stage = 'DONE'
        trim, cost = self.best_trim
        bow = self.base + trim; stern = self.base - trim
        sys.stdout.write("\n" + "=" * 66 + "\n  ИТОГ КАЛИБРОВКИ\n" + "=" * 66 + "\n")
        sys.stdout.write(f"  Нейтраль (база):     bz_neutral = {self.base:.4f}\n")
        sys.stdout.write(f"  Дифферент (trim):    {trim:+.4f}   [{reason}]\n")
        sys.stdout.write(f"  -> носовые баки (1,2):  {bow:.4f}  ({bow*MAX_BALLAST_VOL*1000:.2f} л)\n")
        sys.stdout.write(f"  -> кормовые баки(3,4):  {stern:.4f}  ({stern*MAX_BALLAST_VOL*1000:.2f} л)\n")
        sys.stdout.write(f"  Остаточный тангаж:   {math.degrees(cost):.2f}°\n")
        sys.stdout.write("\n  Впиши в models.py:  bz_neutral = %.3f  (и trim для диффер.) \n" % self.base)
        sys.stdout.write("=" * 66 + "\n"); sys.stdout.flush()
        self._set_trim(self.base, trim)
        raise SystemExit


def main():
    rclpy.init()
    node = BallastCalib()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
