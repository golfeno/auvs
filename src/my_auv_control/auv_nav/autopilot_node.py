#!/usr/bin/env python3
"""AUV Autopilot v101 — мягкий обход, плавный балласт, фильтр входа."""
import sys, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray
from .models import MotorMode, DepthMode, ActuatorCommands, Phys, PID, Lim
from .sensor_fusion import SensorFusion
from .phase_manager import PhaseManager
from .guidance import LOSGuidance
from .heading import HeadingController
from .depth_rudder import DepthRudderController
from .depth_ballast import DepthBallastController
from .roll import RollController
from .telemetry import Telemetry
from .obstacle_avoidance import ObstacleAvoidance
from .version import VERSION, BUILD_NUMBER


class AUVAutopilotNode(Node):
    def __init__(self, wps, mm, dm):
        super().__init__('auv_ctrl')
        self.mm = mm; self.dm = dm; self.tw = len(wps)
        self.pm = PhaseManager(wps, mm, dm)
        self.sf = SensorFusion(self)
        self.los = LOSGuidance()
        self.heading = HeadingController()
        self.depth_r = DepthRudderController()
        self.depth_b = DepthBallastController()
        self.roll = RollController()
        self.obstacle = ObstacleAvoidance()
        self.tl = Telemetry(self)
        _dl = {DepthMode.RUDDER: 'Глубина:РУЛИ',
               DepthMode.BALLAST: 'Глубина:БАКИ',
               DepthMode.BOTH: 'Глубина:БАКИ+РУЛИ'}
        self.tl.depth_label = _dl.get(dm, '')

        self.pub_lt   = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_rt   = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_vert_top = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder_top/cmd_position', 10)
        self.pub_hl   = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_hr   = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)
        self.pub_hfl  = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_front_left/cmd_position', 10)
        self.pub_hfr  = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_front_right/cmd_position', 10)

        self.pub_b = []
        for i in range(1, 5):
            self.pub_b.append(self.create_publisher(
                Float64, f'/model/sub_ballast_{i}/buoyancy_engine', 10))

        self.wps = wps
        self.pub_markers = self.create_publisher(MarkerArray, '/auv/waypoints', 10)
        self.marker_timer = self.create_timer(0.5, self._pub_markers)

        self.depth_schedule = None
        self._b = 0.0
        self.timer = self.create_timer(Phys.DT, self._loop)
        t0 = self.get_clock().now().nanoseconds / 1e9
        self.pm.init_wp(t0, [0.0, 0.0, 0.0])

    def _apply_segment_depth(self):
        if not self.depth_schedule:
            return
        i = self.pm.wp_idx
        if 0 <= i < len(self.depth_schedule):
            dm = self.depth_schedule[i]
            if dm != self.dm:
                self.dm = dm
                _dl = {DepthMode.RUDDER: 'Глубина:РУЛИ',
                       DepthMode.BALLAST: 'Глубина:БАКИ',
                       DepthMode.BOTH: 'Глубина:БАКИ+РУЛИ'}
                self.tl.depth_label = _dl.get(dm, '') + ' (демо)'

    def _loop(self):
        if self.pm.state == 'FINISH':
            return

        self._apply_segment_depth()

        t = self.get_clock().now().nanoseconds / 1e9

        # Sensors
        self.sf.update(self.pm.target, Phys.DT)
        self.pm.update_drift(self.sf.state, t, Phys.DT)

        s = self.sf.state

        # ─── Obstacle ───
        av = None
        s.avoid_mode = 'NORMAL'
        if self.pm.state not in ('HOVER_STAB', 'FINISH'):
            # Порог входа в обход зависит от фазы: на маршевых (круиз/коридор) —
            # дальний (12 м, заранее видим помеху), на ближних (Z_STAB/сближение) —
            # короткий (5 м, не дёргаемся у самой точки).
            if self.pm.state in ('NAV', 'Z_CORRIDOR'):
                self.obstacle.AVOID_RANGE = self.obstacle.AVOID_RANGE_FAR
            else:
                self.obstacle.AVOID_RANGE = self.obstacle.AVOID_RANGE_NEAR
            av = self.obstacle.compute(s.sonar_ranges, s.rpy[2], s.bearing, Phys.DT)
            s.avoid_mode = av.mode
            # диагностика обхода для телеметрии
            s.avoid_closest = av.closest
            s.avoid_yaw = av.yaw_offset
            s.avoid_dir = av._dir if av._dir else '-'
            s.zL = av.zL; s.zR = av.zR; s.zU = av.zU; s.zD = av.zD

        # ─── Фаза ───
        prev_ph = self.pm.state
        ph = self.pm.evaluate(s, t, av, Phys.DT)
        if ph != prev_ph:
            sys.stdout.write(f"\n>>> ФАЗА: {prev_ph} -> {ph}  "
                             f"(closest={av.closest if av else 0:.1f}м "
                             f"yaw_off={av.yaw_offset if av else 0:+.2f})\n")
            sys.stdout.flush()

        # ─── LOS-наведение по прямой A→B ───
        # Держим аппарат на ОТРЕЗКЕ маршрута (а не дугой к точке). seg_start
        # после обхода переустанавливается на текущую позицию (reroute), поэтому
        # LOS ведёт от места, где аппарат разошёлся с препятствием, прямо к цели.
        # Применяем только в маршевых фазах; в APPROACH — наведение на точку.
        self.los.set_segment(self.pm.seg_start, self.pm.target)
        if ph in ('NAV', 'Z_CORRIDOR'):
            self.los.apply(s)        # перепишет s.yaw_err по закону LOS
        else:
            s.cross_track = 0.0

        # VFH-обход активен ТОЛЬКО в фазе AVOID (там рулим на desired_heading).
        # Вне AVOID курс/глубину ведут LOS и контуры цели — поправку не подмешиваем.

        # Reset on phase change
        if self.pm.need_pid_reset:
            self.heading.reset()
            self.depth_r.reset()
            self.depth_b.reset()
            self.roll.reset()
            # ВАЖНО: obstacle.reset() здесь НЕ вызываем — иначе при входе в AVOID
            # стирается память PASS-through (_dir/_pass) и обход бросает манёвр,
            # как только конус сонара теряет объект. Память обхода живёт всю
            # текущую точку; сбрасывается при смене waypoint (arrival/init_wp).
            self._b = 0.0
            self.pm.need_pid_reset = False

        # params
        tp = self.pm.params(s, av)

        cmd = ActuatorCommands()

        # ─── РУЛИ ───
        if ph == 'AVOID' and av is not None and av.has_heading:
            # ── VFH: рулим на АБСОЛЮТНЫЙ desired_heading из обходчика ──
            # Обходчик сам выбрал свободную «долину» ближе к цели и выдал твёрдый
            # абсолютный курс (с коммитом/гистерезисом -> без щёлканья). Курс на
            # цель в фазе AVOID не вмешивается: desired_heading уже учитывает цель.
            import math as _m
            yaw_err_av = _m.atan2(_m.sin(av.desired_heading - s.rpy[2]),
                                  _m.cos(av.desired_heading - s.rpy[2]))
            yaw_cmd = PID.Kp_yaw * yaw_err_av - PID.Kd_yaw * s.yaw_d
            cmd.rv = max(-PID.rud_max, min(PID.rud_max, yaw_cmd))
            # Верхний верт. руль помогает повороту + гасит крен.
            cmd.rvt = max(-PID.roll_v_lim, min(PID.roll_v_lim,
                          yaw_err_av * 1.5 + PID.Kp_roll_v * s.rpy[0]))

            # Глубину держим на цели маршрута (VFH обходит ТОЛЬКО по горизонтали).
            if self.dm in (DepthMode.RUDDER, DepthMode.BOTH):
                cmd.hl, cmd.hr, cmd.hfl, cmd.hfr = self.depth_r.compute(s, ph, Phys.DT)
            if self.dm in (DepthMode.BALLAST, DepthMode.BOTH):
                cmd.ballast_volume = self.depth_b.compute(s, ph, Phys.DT)
        else:
            cmd.rv = self.heading.compute(s, ph, Phys.DT, backoff=False)
            cmd.rvt = self.roll.vertical_top(s, ph, Phys.DT)
            if self.dm in (DepthMode.RUDDER, DepthMode.BOTH):
                cmd.hl, cmd.hr, cmd.hfl, cmd.hfr = self.depth_r.compute(s, ph, Phys.DT)
            if self.dm in (DepthMode.BALLAST, DepthMode.BOTH):
                cmd.ballast_volume = self.depth_b.compute(s, ph, Phys.DT)

        # ─── Тяга ───
        slew_map = {'NAV': 6.0, 'Z_CORRIDOR': 6.0, 'Z_STAB': 5.0,
                    'APPROACH': 8.0, 'AVOID': 8.0, 'HOVER_STAB': 0.0, 'FINISH': 0.0}
        sl = slew_map.get(ph, 6.0)
        d = sl * Phys.DT
        tgt = tp.get('bs', 0.0)

        # Замедление у помехи (чтобы не врезаться): множим тягу на speed_factor
        # ВО ВСЕХ фазах, где помеха близко (в т.ч. AVOID). speed_factor спадает к
        # MIN_SPEED по мере приближения.
        if av is not None and av.obstacle_near:
            tgt *= av.speed_factor
            # У самого объекта (ближе BRAKE_RANGE) НЕ глушим ход в ноль/реверс —
            # держим МИНИМАЛЬНЫЙ ход вперёд (avoid_thrust*MIN_SPEED). Так аппарат
            # не врезается (медленно), но и не виснет: есть поток на рулях.
            if av.closest < self.obstacle.BRAKE_RANGE:
                min_fwd = Lim.avoid_thrust * self.obstacle.MIN_SPEED  # отрицательная = вперёд
                tgt = min(tgt, min_fwd)   # не быстрее, но и не медленнее минимума

        self._b = self._b + max(-d, min(d, tgt - self._b))

        if ph in ('HOVER_STAB', 'FINISH'):
            cmd.lt = 0.0; cmd.rt = 0.0
        else:
            # Дифференциал движков: курс рулится разницей тяги (единственный режим).
            yd = tp.get('yd', 0.0)
            cmd.lt = self._b + yd
            cmd.rt = self._b - yd

        # Publish
        self._pub(cmd)
        self.tl.analytics(s, ph, self.pm.wp_idx)

        # Arrival
        if ph == 'HOVER_STAB' and abs(s.vel) < 0.25 and abs(s.z_err) < 0.5:
            self.tl.log_wp(self.pm.wp_idx + 1)
            self.pm.wp_idx += 1
            if self.pm.wp_idx < self.tw:
                self.heading.reset(); self.depth_r.reset(); self.depth_b.reset()
                self.roll.reset(); self.obstacle.reset()
                self._b = 0.0
                self.pm.init_wp(t, s.pos)
            else:
                self.pm.state = 'FINISH'
                self._pub(ActuatorCommands())
                sys.stdout.write(f"\n[{VERSION} #{BUILD_NUMBER}] ✓ Миссия завершена!\n")
                sys.stdout.flush()
                raise SystemExit
        else:
            self.tl.log(s, ph, self.pm.wp_idx, t, self.tw, cmd)

    def _pub_markers(self):
        arr = MarkerArray()
        for i, w in enumerate(self.wps):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'waypoints'; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = float(w[0]); m.pose.position.y = float(w[1]); m.pose.position.z = float(w[2])
            m.pose.orientation.w = 1.0
            active = (i == self.pm.wp_idx)
            sc = 1.2 if active else 0.7
            m.scale.x = sc; m.scale.y = sc; m.scale.z = sc
            if active:
                m.color.r, m.color.g, m.color.b = 0.1, 0.9, 0.2
            elif i < self.pm.wp_idx:
                m.color.r, m.color.g, m.color.b = 0.4, 0.4, 0.4
            else:
                m.color.r, m.color.g, m.color.b = 0.95, 0.55, 0.1
            m.color.a = 0.8
            arr.markers.append(m)
            txt = Marker()
            txt.header.frame_id = 'world'; txt.header.stamp = m.header.stamp
            txt.ns = 'wp_labels'; txt.id = 1000 + i
            txt.type = Marker.TEXT_VIEW_FACING; txt.action = Marker.ADD
            txt.pose.position.x = float(w[0]); txt.pose.position.y = float(w[1])
            txt.pose.position.z = float(w[2]) + 1.0
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.8
            txt.color.r = txt.color.g = txt.color.b = 1.0; txt.color.a = 0.9
            txt.text = str(i + 1)
            arr.markers.append(txt)
        self.pub_markers.publish(arr)

    def _pub(self, cmd):
        self.pub_lt.publish(Float64(data=cmd.lt))
        self.pub_rt.publish(Float64(data=cmd.rt))
        self.pub_vert.publish(Float64(data=cmd.rv))
        self.pub_vert_top.publish(Float64(data=cmd.rvt))
        self.pub_hl.publish(Float64(data=cmd.hl))
        self.pub_hr.publish(Float64(data=cmd.hr))
        self.pub_hfl.publish(Float64(data=cmd.hfl))
        self.pub_hfr.publish(Float64(data=cmd.hfr))
        if self.dm in (DepthMode.BALLAST, DepthMode.BOTH):
            base = cmd.ballast_volume
        else:
            base = Phys.RUDDER_TRIM_VOL
        tr = Phys.BALLAST_TRIM
        bow = max(0.0, min(1.0, base + tr)) * Phys.MAX_BALLAST_VOL
        stern = max(0.0, min(1.0, base - tr)) * Phys.MAX_BALLAST_VOL
        self.pub_b[0].publish(Float64(data=bow))
        self.pub_b[1].publish(Float64(data=bow))
        self.pub_b[2].publish(Float64(data=stern))
        self.pub_b[3].publish(Float64(data=stern))


def load_waypoints(filepath):
    wps = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) == 3:
                wps.append(tuple(map(float, parts)))
    return wps


def _ask(p, v, d=None):
    while True:
        r = input(p).strip()
        if r == '' and d is not None:
            return d
        try:
            n = int(r)
            if n in v:
                return n
        except ValueError:
            pass


def main():
    print("=" * 50)
    print(f"  AUV Autopilot {VERSION} | СБОРКА #{BUILD_NUMBER}")
    print("=" * 50)

    default_file = os.path.expanduser("~/auv/waypoints.txt")
    wps = []
    if os.path.exists(default_file):
        print(f"\n  Файл: {default_file}")
        use = input("  Использовать? [Y/n]: ").strip().lower()
        if use in ('', 'y', 'yes'):
            wps = load_waypoints(default_file)
            print(f"  Загружено {len(wps)} точек:")
            for i, w in enumerate(wps, 1):
                print(f"    {i}: ({w[0]}, {w[1]}, {w[2]})")

    if not wps:
        print("\nWaypoints (X Y Z). Пустая = старт.")
        i = 1
        while True:
            try:
                inp = input(f"  Точка {i}: ").strip()
                if not inp:
                    if wps: break
                    continue
                p = inp.split()
                if len(p) == 3:
                    wps.append(tuple(map(float, p))); i += 1
            except ValueError:
                pass

    motor = MotorMode.DUAL   # единственный режим (дифференциал движков)

    dm = _ask("Глубина [1=рули / 2=балласты / 3=оба] (1): ", {1, 2, 3}, 1)
    depth = {1: DepthMode.RUDDER, 2: DepthMode.BALLAST, 3: DepthMode.BOTH}[dm]
    print(f"\n  → Двигатели: {motor.name}  |  Глубина: {depth.name}")

    rclpy.init()
    node = AUVAutopilotNode(wps, motor, depth)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node._pub(ActuatorCommands())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
