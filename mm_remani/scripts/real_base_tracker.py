#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class RealBaseTracker:
    def __init__(self):
        rospy.init_node('real_base_tracker', anonymous=True)

        # 1. 发布给实车底盘的速度控制指令
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.err_pub = rospy.Publisher('/tracking_error', Float64MultiArray, queue_size=10)

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

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False

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
        self.allow_reverse = rospy.get_param('~allow_reverse', False)
        self.command_timeout = max(0.05, rospy.get_param('~command_timeout', 0.3))
        self.target_stamp = rospy.Time(0)
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

    def car_cmd_cb(self, msg):
        self.target_x = msg.linear.x
        self.target_y = msg.linear.y
        self.target_yaw = msg.linear.z
        self.target_vx = msg.angular.x
        self.target_vy = msg.angular.y
        self.target_w  = msg.angular.z
        self.has_target = True
        self.target_stamp = rospy.Time.now()

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self, event):
        if not self.has_odom or not self.has_target:
            self.publish_stop()
            return

        if (rospy.Time.now() - self.target_stamp).to_sec() > self.command_timeout:
            rospy.logwarn_throttle(1.0, "Trajectory command timed out; stopping base")
            self.publish_stop()
            return

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

        # 如果已经非常接近终点，直接刹车停稳
        if dist < 0.15 and abs(eyaw) < 0.1 and abs(v_r) < 0.05:
            err_msg = Float64MultiArray()
            err_msg.data = [dist, ex, ey, eyaw]
            self.err_pub.publish(err_msg)
            self.publish_stop()
            return

        # ========================================================
        # 控制律: 角速度前馈 + P纠正
        # cmd.angular.z = omega_des + k_yaw * eyaw + k_y * |v| * ey
        # ========================================================
        cmd = Twist()
        cmd.linear.x = v_r * math.cos(eyaw) + self.k_x * ex
        cmd.angular.z = self.target_w + self.k_yaw * eyaw + self.k_y * v_r * ey

        # 发布诊断数据
        err_msg = Float64MultiArray()
        err_msg.data = [dist, ex, ey, eyaw]
        self.err_pub.publish(err_msg)

        # 安全限幅
        if cmd.linear.x > self.max_v:  cmd.linear.x = self.max_v
        if cmd.linear.x < -self.max_v: cmd.linear.x = -self.max_v
        if cmd.angular.z > self.max_w: cmd.angular.z = self.max_w
        if cmd.angular.z < -self.max_w: cmd.angular.z = -self.max_w

        # The real base is configured for forward-only motion.  This remains a
        # final safety guard if an old trajectory or another publisher requests reverse.
        if not self.allow_reverse and cmd.linear.x < 0.0:
            rospy.logwarn_throttle(1.0, "Blocked reverse cmd_vel request")
            cmd.linear.x = 0.0

        # 下发给底盘驱动节点
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        RealBaseTracker()
    except rospy.ROSInterruptException:
        pass
