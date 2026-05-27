#!/usr/bin/env python3
"""AUV Autopilot v49.10 | Technical Phase Names & Multi-line Logging"""
import rclpy, math, time, sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64, String
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

P_Z0 = 101325.0
RHO_G = 9810.0

PHASE_TRANSLATION = {
    'NAV': 'Круиз',
    'Z_STAB': 'Коррекция высоты',
    'XY_FINAL': 'Сближение',
    'HOVER_STAB': 'Стабилизация',
    'FINISH': 'Готово'
}

class AUVMultiLineAutopilot(Node):
    def __init__(self, waypoint_list):
        super().__init__('auv_ctrl')
        self.waypoints = waypoint_list
        self.current_wp_idx = 0
        
        self.pub_lt = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_rt = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_hl = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_hr = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)
        
        self.pub_p_nav = self.create_publisher(Point, '/analytics/phase_nav', 10)
        self.pub_p_stab = self.create_publisher(Point, '/analytics/phase_stab', 10)
        self.pub_status = self.create_publisher(String, '/model/submarine/status', 10)

        self.create_subscription(Odometry, '/model/submarine/odometry', self.odom_cb, 10)
        qos_s = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32, '/model/submarine/pressure', self.press_cb, qos_s)

        self.state = 'INIT'
        self.pos = [0.0, 0.0, 0.0]  
        self.baro_z = 0.0
        self.vel = 0.0
        self.rpy = [0.0, 0.0, 0.0]
        self.prev_rpy = [0.0, 0.0, 0.0]
        self.target_global = [0.0, 0.0, 0.0]
        self.bearing = 0.0
        self.dist_2d = 1000.0
        self.dist_3d = 1000.0
        self.prev_baro_z = 0.0
        self.dz_dt = 0.0
        
        self.curr_rv = 0.0
        self.curr_hl = 0.0
        self.curr_hr = 0.0
        self.curr_cmd_base = 0.0 
        
        self.max_rudder_speed = 2.4 
        self.max_cruise_speed = -35.0  
        self.min_cruise_speed = -12.0  
        
        self.success_radius = 0.85  
        self.altitude_breach_threshold = 1.7  
        self.is_re_stabilizing = False        
        
        self.Kp_z_base = 18.0; self.Kd_z_base = 22.0      
        self.Kp_yaw = 5.0; self.Kd_yaw = 2.8 
        self.Kp_roll = 50.0; self.Kd_roll = 22.0 
        self.roll_bias = 0.04
        
        self.xy_final_start_time = 0.0
        self.in_back_off_maneuver = False
        self.back_off_start_time = 0.0
        self.reset_performed_for_current_wp = False
        
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.loop)

    def press_cb(self, msg):
        self.baro_z = (P_Z0 - msg.data) / RHO_G

    def odom_cb(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = self.baro_z 
        self.vel = msg.twist.twist.linear.x
        
        q = msg.pose.pose.orientation
        self.rpy[0] = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        self.rpy[1] = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        self.rpy[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        
        if self.state == 'INIT':
            self.init_new_waypoint()

        dx_rem = self.target_global[0] - self.pos[0]
        dy_rem = self.target_global[1] - self.pos[1]
        dz_rem = self.target_global[2] - self.pos[2]
        
        self.dist_2d = math.hypot(dx_rem, dy_rem)
        self.dist_3d = math.sqrt(dx_rem**2 + dy_rem**2 + dz_rem**2)
        self.bearing = math.atan2(dy_rem, dx_rem)
            
        p = Point(x=self.pos[0], y=self.pos[1], z=self.pos[2])
        if self.state in ['NAV', 'XY_FINAL']:
            self.pub_p_nav.publish(p)
        elif self.state in ['Z_STAB', 'HOVER_STAB']:
            self.pub_p_stab.publish(p)

    def init_new_waypoint(self):
        self.target_global = list(self.waypoints[self.current_wp_idx])
        self.prev_rpy = list(self.rpy)
        self.prev_baro_z = self.baro_z
        self.dz_dt = 0.0
        self.in_back_off_maneuver = False
        self.reset_performed_for_current_wp = False
        self.is_re_stabilizing = False
        self.xy_final_start_time = self.get_clock().now().nanoseconds / 1e9
        
        z_err_initial = self.pos[2] - self.target_global[2]
        if abs(z_err_initial) > 2.2:
            self.state = 'Z_STAB'
        else:
            self.state = 'NAV'

    def constrain_slew(self, current, target, max_rate):
        max_change = max_rate * self.dt
        error = target - current
        return current + max(-max_change, min(max_change, error))

    def loop(self):
        if self.state not in ['NAV', 'Z_STAB', 'XY_FINAL', 'HOVER_STAB', 'FINISH']: return
        
        t_now = self.get_clock().now().nanoseconds / 1e9
        status_msg = String()
        status_msg.data = f"WP_{self.current_wp_idx + 1}_{self.state}"
        self.pub_status.publish(status_msg)

        z_err = self.pos[2] - self.target_global[2] 
        
        raw_dz = (self.pos[2] - self.prev_baro_z) / self.dt
        self.dz_dt = 0.6 * self.dz_dt + 0.4 * raw_dz
        self.prev_baro_z = self.pos[2]
        
        roll_abs = abs(self.rpy[0])
        pitch_curr = self.rpy[1]

        # === АКТИВНЫЙ ПЕРЕХВАТЧИК ВЫСОТЫ ===
        if self.state in ['XY_FINAL', 'HOVER_STAB'] and abs(z_err) > self.altitude_breach_threshold:
            self.state = 'Z_STAB'
            self.is_re_stabilizing = True  
            self.in_back_off_maneuver = False
            self.xy_final_start_time = t_now

        # === ДИНАМИЧЕСКИЙ ГЕЙН-ШЕДУЛИНГ ===
        vel_abs = max(0.1, abs(self.vel))
        
        if self.state == 'XY_FINAL':
            Kp_z = self.Kp_z_base * 0.8
            Kd_z = self.Kd_z_base * 1.4
        else:
            vel_scale = max(0.4, min(1.0, vel_abs / 2.5))
            Kp_z = self.Kp_z_base * vel_scale
            Kd_z = self.Kd_z_base * vel_scale

        roll_damping = 1.0 - max(0.0, min(0.6, roll_abs * 2.0))
        target_rudder_h = -(Kp_z * z_err + Kd_z * self.dz_dt) * roll_damping
        
        pitch_limit = 0.25 if self.state == 'XY_FINAL' else 0.45
        if pitch_curr > pitch_limit:   
            target_rudder_h = min(target_rudder_h, -0.2)
        elif pitch_curr < -pitch_limit: 
            target_rudder_h = max(target_rudder_h, 0.2)

        target_rudder_h = max(-0.55, min(0.55, target_rudder_h))

        # === КУРС ===
        yaw_err = math.atan2(math.sin(self.bearing - self.rpy[2]), math.cos(self.bearing - self.rpy[2]))
        d_yaw = (self.rpy[2] - self.prev_rpy[2]) / self.dt
        
        if self.in_back_off_maneuver and (t_now - self.back_off_start_time) < 1.5:
            target_rudder_v = 0.0
        else:
            target_rudder_v = self.Kp_yaw * yaw_err + self.Kd_yaw * d_yaw
            if roll_abs > 0.18 and self.state == 'XY_FINAL':
                target_rudder_v *= 0.35
            target_rudder_v = max(-0.5, min(0.5, target_rudder_v))

        # === КРЕН ===
        roll_err = self.rpy[0]
        d_roll = (self.rpy[0] - self.prev_rpy[0]) / self.dt
        roll_pid = self.Kp_roll * roll_err + self.Kd_roll * d_roll
        
        pitch_priority = max(0.15, 1.0 - (abs(z_err) / 5.0))
        roll_pid *= pitch_priority 
        
        raw_hl = max(-0.95, min(0.95, target_rudder_h - roll_pid - self.roll_bias))
        raw_hr = max(-0.95, min(0.95, target_rudder_h + roll_pid + self.roll_bias))
        self.prev_rpy = list(self.rpy)
        
        self.curr_rv = self.constrain_slew(self.curr_rv, target_rudder_v, self.max_rudder_speed)
        self.curr_hl = self.constrain_slew(self.curr_hl, raw_hl, self.max_rudder_speed)
        self.curr_hr = self.constrain_slew(self.curr_hr, raw_hr, self.max_rudder_speed)
        
        cmd_lt = 0.0; cmd_rt = 0.0

        # === АВТОМАТ ФАЗ ===
        if self.state == 'NAV':
            scale = min(1.0, self.dist_2d / 15.0)
            target_base = self.max_cruise_speed * scale
            if roll_abs > 0.15: target_base *= 0.55
                
            self.curr_cmd_base = self.constrain_slew(self.curr_cmd_base, target_base, 15.0)
            yaw_diff = 5.0 * yaw_err
            cmd_lt = self.curr_cmd_base + yaw_diff
            cmd_rt = self.curr_cmd_base - yaw_diff
            if self.dist_2d < 2.5: self.state = 'Z_STAB'

        elif self.state == 'Z_STAB':
            if self.is_re_stabilizing:
                target_base = -18.0 if abs(z_err) > 1.4 else -8.0
            else:
                target_base = -16.0
                
            if roll_abs > 0.15: target_base *= 0.6
                
            self.curr_cmd_base = self.constrain_slew(self.curr_cmd_base, target_base, 14.0)
            yaw_diff = 5.0 * yaw_err
            cmd_lt = self.curr_cmd_base + yaw_diff
            cmd_rt = self.curr_cmd_base - yaw_diff
            
            look_ahead_time = 1.0 
            predicted_z_err = z_err + (self.dz_dt * look_ahead_time)
            
            if abs(predicted_z_err) < 1.2 and abs(z_err) < 1.0 and abs(self.dz_dt) < 0.14:
                if self.dist_2d > 3.5 and not self.is_re_stabilizing:
                    self.state = 'NAV'
                else:
                    self.state = 'XY_FINAL'
                    self.is_re_stabilizing = False  
                    self.xy_final_start_time = t_now
                    self.in_back_off_maneuver = False

        elif self.state == 'XY_FINAL':
            if not self.in_back_off_maneuver and not self.reset_performed_for_current_wp and (t_now - self.xy_final_start_time) > 9.0:
                self.in_back_off_maneuver = True
                self.reset_performed_for_current_wp = True 
                self.back_off_start_time = t_now
            
            if self.in_back_off_maneuver:
                target_base = 20.0 
                self.curr_cmd_base = self.constrain_slew(self.curr_cmd_base, target_base, 18.0)
                cmd_lt = self.curr_cmd_base
                cmd_rt = self.curr_cmd_base
                
                if self.dist_2d > 3.0 or (t_now - self.back_off_start_time) > 2.5:
                    self.in_back_off_maneuver = False
                    self.xy_final_start_time = t_now 
            else:
                target_base = -11.0
                if roll_abs > 0.10: target_base *= 0.5
                    
                self.curr_cmd_base = self.constrain_slew(self.curr_cmd_base, target_base, 8.0)
                yaw_diff = min(4.5, max(-4.5, 5.0 * yaw_err))
                cmd_lt = self.curr_cmd_base + yaw_diff
                cmd_rt = self.curr_cmd_base - yaw_diff

            if self.dist_2d <= self.success_radius and abs(z_err) <= 0.85:
                self.state = 'HOVER_STAB'

        elif self.state == 'HOVER_STAB':
            cmd_lt = 0.0; cmd_rt = 0.0
            if abs(self.vel) < 0.18:
                # Фиксируем строку при достижении точки (добавляется перевод строки \n)
                ru_phase = PHASE_TRANSLATION.get(self.state, self.state)
                sys.stdout.write(
                    f"\r[v49.10] Тчк:{self.current_wp_idx+1} | Фаза: {ru_phase} | "
                    f"XYZ: ({self.pos[0]:.1f}, {self.pos[1]:.1f}, {self.pos[2]:.1f}) | "
                    f"V: {self.vel:+.2f}м/с | D2D: {self.dist_2d:.2f}м | D3D: {self.dist_3d:.2f}м | Z_Err: {z_err:+.2f}м\n"
                )
                sys.stdout.flush()

                if self.current_wp_idx + 1 < len(self.waypoints):
                    self.current_wp_idx += 1
                    self.init_new_waypoint()
                else:
                    self.state = 'FINISH'
                    self._pub(0,0,0,0,0)
                    sys.stdout.write(f"[v49.10] Миссия успешно завершена!\n")
                    sys.stdout.flush()
                    raise SystemExit

        self._pub(cmd_lt, cmd_rt, self.curr_rv, self.curr_hl, self.curr_hr)
        
        # Интерактивное обновление лога текущей точки на одной строке через \r
        if self.state != 'FINISH':
            ru_phase = PHASE_TRANSLATION.get(self.state, self.state)
            sys.stdout.write(
                f"\r[v49.10] Тчк:{self.current_wp_idx+1} | Фаза: {ru_phase} | "
                f"XYZ: ({self.pos[0]:.1f}, {self.pos[1]:.1f}, {self.pos[2]:.1f}) | "
                f"V: {self.vel:+.2f}м/с | D2D: {self.dist_2d:.2f}м | D3D: {self.dist_3d:.2f}м | Z_Err: {z_err:+.2f}м"
            )
            sys.stdout.flush()

    def _pub(self, lt, rt, rv, hl, hr):
        self.pub_lt.publish(Float64(data=float(lt)))
        self.pub_rt.publish(Float64(data=float(rt)))
        self.pub_vert.publish(Float64(data=float(rv)))
        self.pub_hl.publish(Float64(data=float(hl)))
        self.pub_hr.publish(Float64(data=float(hr)))

    def run(self):
        try: rclpy.spin(self)
        except (KeyboardInterrupt, SystemExit): self._pub(0,0,0,0,0)

def main():
    wps = []
    wp_count = 1
    while True:
        try:
            inp = input(f"Точка {wp_count} (X Y Z) или Enter: ").strip()
            if not inp:
                if not wps: continue
                break
            parts = inp.split()
            if len(parts) != 3: continue
            x, y, z = map(float, parts)
            wps.append((x, y, z))
            wp_count += 1
        except ValueError: pass

    print(f"[v49.10] Запуск...")
    rclpy.init()
    node = AUVMultiLineAutopilot(wps)
    try: node.run()
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
