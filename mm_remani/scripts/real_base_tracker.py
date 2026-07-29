#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Float64MultiArray

class RealBaseTracker:
    def __init__(self):
        rospy.init_node('real_base_tracker', anonymous=True)

        # 1. 发布给实车底盘的速度控制指令
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.err_pub = rospy.Publisher('/tracking_error', Float64MultiArray, queue_size=10)
        self.fault_pub = rospy.Publisher('/tracking_fault', Bool, queue_size=1, latch=True)
        self.safety_speed_limit = None

        # 2. 接收由 REMANI 规划器发出的期望轨迹点（包裹在 Twist 里）
        #    linear.x = 目标X坐标
        #    linear.y = 目标Y坐标
        #    linear.z = 目标Yaw偏航角
        #    angular.x = 目标线速度X
        #    angular.y = 目标线速度Y
        #    angular.z = 期望角速度 (omega feedforward)
        self.cmd_sub = rospy.Subscriber('/mm_controller_node/car_cmd', Twist, self.car_cmd_cb)

        # 3. 接收刚才写的实车的实时定位Odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.safety_speed_sub = rospy.Subscriber(
            '/planning/safety_speed_limit', Float64, self.safety_speed_limit_cb)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        self.odom_receive_stamp = rospy.Time(0)
        self.odom_header_stamp = rospy.Time(0)

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_w  = 0.0
        self.has_target = False

        # =======================================================
        # 实车轨迹跟踪 P 参数
        # =======================================================
        self.k_x = 1.5     # 前进方向误差纠正（降低减少震荡）
        self.k_y = 1.0     # 侧向偏离纠正（降低减少过调）
        self.k_yaw = 1.5   # 转角纠正（降低减少剧烈拐弯）

        # 最大限幅将从统配文件 (`real_speed.yaml`) 中的参数服务器拉取
        self.max_v = rospy.get_param('~tracker_max_v', 0.6)   # 最大线速度
        self.max_w = rospy.get_param('~tracker_max_w', 1.5)   # 最大角速度
        self.max_linear_accel = max(0.01, rospy.get_param('~tracker_max_linear_accel', 0.35))
        self.max_angular_accel = max(0.01, rospy.get_param('~tracker_max_angular_accel', 1.0))
        self.allow_reverse = rospy.get_param('~allow_reverse', False)
        self.command_timeout = max(0.05, rospy.get_param('~command_timeout', 0.3))
        self.odom_timeout = max(0.05, rospy.get_param('~odom_timeout', 0.25))
        self.odom_header_timeout = max(
            self.odom_timeout, rospy.get_param('~odom_header_timeout', 0.25))
        self.tracking_stop_distance = max(
            0.05, rospy.get_param('~tracking_stop_distance', 0.18))
        self.tracking_stop_yaw = max(
            0.1, rospy.get_param('~tracking_stop_yaw', 0.50))
        self.tracking_recover_distance = min(
            self.tracking_stop_distance,
            max(0.02, rospy.get_param('~tracking_recover_distance', 0.08)))
        self.tracking_recover_yaw = min(
            self.tracking_stop_yaw,
            max(0.05, rospy.get_param('~tracking_recover_yaw', 0.25)))
        self.tracking_recover_frames = max(
            1, int(rospy.get_param('~tracking_recover_frames', 5)))
        self.goal_position_tolerance = max(0.01, rospy.get_param('~goal_position_tolerance', 0.05))
        self.goal_yaw_tolerance = max(0.01, rospy.get_param('~goal_yaw_tolerance', 0.1))
        self.goal_speed_tolerance = max(0.0, rospy.get_param('~goal_speed_tolerance', 0.05))
        self.local_reverse_recovery = rospy.get_param('~local_reverse_recovery', True)
        self.local_reverse_max_distance = max(0.02, rospy.get_param('~local_reverse_max_distance', 0.15))
        self.local_reverse_max_lateral_error = max(0.01, rospy.get_param('~local_reverse_max_lateral_error', 0.08))
        self.local_reverse_max_yaw_error = max(0.01, rospy.get_param('~local_reverse_max_yaw_error', 0.35))
        self.local_reverse_max_speed = max(0.01, rospy.get_param('~local_reverse_max_speed', 0.08))
        self.local_reverse_max_travel = max(
            0.05, rospy.get_param('~local_reverse_max_travel', 0.70))
        self.target_stamp = rospy.Time(0)
        self.last_cmd = Twist()
        self.last_control_stamp = rospy.Time(0)
        self.reverse_motion_origin = None
        self.tracking_fault = False
        self.tracking_recover_count = 0
        self.fault_pub.publish(Bool(data=False))
        rospy.on_shutdown(self.publish_stop)

        # 控制循环频率 (50ms = 20Hz)
        rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("实车轨迹追踪器(Tracker)已启动，正在将 car_cmd 转换为实车 cmd_vel...")
        rospy.spin()

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True
        self.odom_receive_stamp = rospy.Time.now()
        self.odom_header_stamp = msg.header.stamp

    def car_cmd_cb(self, msg):
        self.target_x = msg.linear.x
        self.target_y = msg.linear.y
        self.target_yaw = msg.linear.z
        self.target_vx = msg.angular.x
        self.target_vy = msg.angular.y
        self.target_w  = msg.angular.z
        self.has_target = True
        self.target_stamp = rospy.Time.now()

    def safety_speed_limit_cb(self, msg):
        if not math.isfinite(msg.data) or msg.data < 0.0:
            self.safety_speed_limit = None
            return
        self.safety_speed_limit = max(0.0, msg.data)

    def publish_stop(self):
        self.last_cmd = Twist()
        self.last_control_stamp = rospy.Time.now()
        self.cmd_pub.publish(self.last_cmd)

    def set_tracking_fault(self, active, reason=""):
        if active == self.tracking_fault:
            return
        self.tracking_fault = active
        self.tracking_recover_count = 0
        self.fault_pub.publish(Bool(data=active))
        if active:
            rospy.logerr("Tracking safety stop: %s", reason)
        else:
            rospy.loginfo("Tracking recovered on a trajectory aligned with current odometry")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def limit_rate(target, previous, max_rate, dt):
        delta = max_rate * max(0.0, dt)
        return max(previous - delta, min(previous + delta, target))

    def control_loop(self, event):
        if not self.has_odom or not self.has_target:
            self.publish_stop()
            return

        now = rospy.Time.now()
        odom_receive_age = (now - self.odom_receive_stamp).to_sec()
        odom_header_age = (
            (now - self.odom_header_stamp).to_sec()
            if not self.odom_header_stamp.is_zero() else 0.0)
        if (odom_receive_age > self.odom_timeout or
                odom_header_age > self.odom_header_timeout):
            self.set_tracking_fault(
                True,
                "odom stale (receive %.3fs, header %.3fs)" %
                (odom_receive_age, odom_header_age))
            self.publish_stop()
            return

        if (now - self.target_stamp).to_sec() > self.command_timeout:
            rospy.logwarn_throttle(1.0, "Trajectory command timed out; stopping base")
            self.publish_stop()
            return

        if self.last_control_stamp.is_zero():
            dt = 0.05
        else:
            dt = min(0.2, max(0.0, (now - self.last_control_stamp).to_sec()))
        self.last_control_stamp = now

        # Target local feedforward velocity
        v_r = self.target_vx * math.cos(self.target_yaw) + self.target_vy * math.sin(self.target_yaw)

        # 全局坐标下的误差计算
        ex_global = self.target_x - self.current_x
        ey_global = self.target_y - self.current_y

        # 转换到车体局部坐标系下的前后、左右误差
        ex = math.cos(self.current_yaw) * ex_global + math.sin(self.current_yaw) * ey_global
        ey = -math.sin(self.current_yaw) * ex_global + math.cos(self.current_yaw) * ey_global
        eyaw = self.normalize_angle(self.target_yaw - self.current_yaw)

        dist = math.hypot(ex_global, ey_global)
        err_msg = Float64MultiArray()
        err_msg.data = [dist, ex, ey, eyaw]
        self.err_pub.publish(err_msg)

        # A severely displaced robot must not cut across the optimized path to
        # chase a future reference point. The planner receives this latched
        # fault and generates a replacement trajectory from measured odometry.
        if self.tracking_fault:
            if (dist <= self.tracking_recover_distance and
                    abs(eyaw) <= self.tracking_recover_yaw):
                self.tracking_recover_count += 1
                if self.tracking_recover_count >= self.tracking_recover_frames:
                    self.set_tracking_fault(False)
            else:
                self.tracking_recover_count = 0
            if self.tracking_fault:
                self.publish_stop()
                return
        elif (dist > self.tracking_stop_distance or
              abs(eyaw) > self.tracking_stop_yaw):
            self.set_tracking_fault(
                True,
                "reference error %.3fm / %.3frad exceeds %.3fm / %.3frad" %
                (dist, abs(eyaw), self.tracking_stop_distance,
                 self.tracking_stop_yaw))
            self.publish_stop()
            return

        # 如果已经非常接近终点，直接刹车停稳
        if (dist < self.goal_position_tolerance and
                abs(eyaw) < self.goal_yaw_tolerance and
                abs(v_r) < self.goal_speed_tolerance):
            self.publish_stop()
            return

        # ========================================================
        # 控制律: 角速度前馈 + P纠正
        # cmd.angular.z = omega_des + k_yaw * eyaw + k_y * |v| * ey
        # ========================================================
        cmd = Twist()
        cmd.linear.x = v_r * math.cos(eyaw) + self.k_x * ex
        cmd.angular.z = self.target_w + self.k_yaw * eyaw + self.k_y * v_r * ey

        # 安全限幅
        if cmd.linear.x > self.max_v:  cmd.linear.x = self.max_v
        if cmd.linear.x < -self.max_v: cmd.linear.x = -self.max_v
        if cmd.angular.z > self.max_w: cmd.angular.z = self.max_w
        if cmd.angular.z < -self.max_w: cmd.angular.z = -self.max_w

        # Keep normal motion forward-only. A small reverse command is allowed only
        # to recover a nearby handoff error; planned long reverse segments stay blocked.
        if not self.allow_reverse and cmd.linear.x < 0.0:
            if self.reverse_motion_origin is None:
                self.reverse_motion_origin = (self.current_x, self.current_y)
            reverse_travel = math.hypot(
                self.current_x - self.reverse_motion_origin[0],
                self.current_y - self.reverse_motion_origin[1])
            local_reverse_ok = (
                self.local_reverse_recovery and
                -self.local_reverse_max_distance <= ex < 0.0 and
                abs(ey) <= self.local_reverse_max_lateral_error and
                abs(eyaw) <= self.local_reverse_max_yaw_error and
                reverse_travel <= self.local_reverse_max_travel
            )
            if local_reverse_ok:
                cmd.linear.x = max(cmd.linear.x, -self.local_reverse_max_speed)
                rospy.loginfo_throttle(1.0, "Using local reverse recovery for trajectory handoff")
            else:
                rospy.logwarn_throttle(1.0, "Blocked non-local reverse cmd_vel request")
                cmd.linear.x = 0.0
        elif cmd.linear.x >= 0.0:
            self.reverse_motion_origin = None

        if self.safety_speed_limit is not None:
            if self.safety_speed_limit <= 1e-3:
                self.publish_stop()
                return
            else:
                cmd.linear.x = max(-self.safety_speed_limit,
                                   min(self.safety_speed_limit, cmd.linear.x))

        # Keep trajectory handoffs physically smooth. The polynomial trajectory
        # already respects planning limits; this only bounds tracker corrections.
        cmd.linear.x = self.limit_rate(cmd.linear.x, self.last_cmd.linear.x,
                                       self.max_linear_accel, dt)
        cmd.angular.z = self.limit_rate(cmd.angular.z, self.last_cmd.angular.z,
                                        self.max_angular_accel, dt)
        self.last_cmd = cmd

        # 下发给底盘驱动节点
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        RealBaseTracker()
    except rospy.ROSInterruptException:
        pass
