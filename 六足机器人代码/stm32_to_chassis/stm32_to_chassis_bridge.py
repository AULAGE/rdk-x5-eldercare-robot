#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stm32_to_chassis_bridge.py

STM32底盘协议 → ROS小车轮式底盘的转换桥

协议解析（STM32 chassis.c）：
  帧格式：0x7B + 0x00 + 0x00 + vx_h/vx_l + vy_h/vy_l + wz_h/wz_l + BCC + 0x7D
  - vx: float m/s * 1000 → int16 (范围 ±30 m/s → ±30000)
  - wz: float rad/s * 1000 → int16 (范围 ±30 rad/s → ±30000)
  波特率：115200

使用方式：
  ros2 run stm32_to_chassis stm32_to_chassis_bridge --ros-args -p port:=/dev/ttyUSB0
"""

import math
import threading

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist


FRAME_HEADER = 0x7B
FRAME_TAIL   = 0x7D
FRAME_LEN    = 11

# 速度阈值（与STM32侧一致）
MOVE_THRESHOLD = 0.05   # m/s
TURN_THRESHOLD = 0.10   # rad/s

# 动作切换防抖
SWITCH_CONFIRM_TIME = 0.3  # 目标动作需稳定保持0.3s才允许切换
ACTION_COOLDOWN     = 1.0  # 冷却时间

# 步态间隔
STEP_DURATION = 0.08  # 秒


class Stm32ToChassisBridge(Node):
    def __init__(self):
        super().__init__('stm32_to_chassis_bridge')

        self.nums1 = ['B', 'C', 'E', 'F', 'H', 'I', 'K', 'L', 'N', 'O', 'Q', 'R']
        self.nums2 = ['A', 'D', 'G', 'J', 'M', 'P']

        # 动作控制
        self.interrupt_event = threading.Event()
        self.current_action  = '停止'
        self.target_action   = '停止'
        self.last_action_change = 0.0
        self.pending_action  = '停止'
        self.pending_since   = self.get_clock().now().nanoseconds / 1e9
        self.action_thread   = None
        self.thread_lock     = threading.Lock()

        # 里程计变量
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_time = self.get_clock().now()
        self.current_vx = 0.0
        self.current_vz = 0.0

        # 串口
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.ser = None
        self._init_serial()

        # 串口接收线程
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        # 发布者
        self.odom_pub = self.create_publisher(
            Odometry, '/odom_combined', 10
        ) if 'Odometry' in globals() else None

        # TF发布
        from tf2_ros import TransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅者（也可以接收cmd_vel作为备用控制）
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )

        # 定时器
        self.control_timer = self.create_timer(0.05, self._control_loop)
        self.odom_timer    = self.create_timer(0.05, self._publish_odom)

        self._init_pose()
        self.get_logger().info('=== STM32→底盘桥接器已启动 ===')
        self.get_logger().info(f'协议: STM32 chassis (115200, 0x7B...0x7D)')
        self.get_logger().info(f'动作防抖: {SWITCH_CONFIRM_TIME}s, 冷却: {ACTION_COOLDOWN}s')

    def _init_serial(self):
        port     = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1.0)
            self.get_logger().info(f'串口初始化成功: {port}, {baudrate} bps')
        except Exception as e:
            self.get_logger().error(f'串口初始化失败: {e}')

    def _bcc_checksum(self, data):
        bcc = 0
        for b in data:
            bcc ^= b
        return bcc & 0xFF

    def _rx_loop(self):
        """后台接收STM32串口数据，解析速度帧并更新目标动作"""
        if not self.ser:
            return
        buf = bytearray()
        while rclpy.ok():
            try:
                d = self.ser.read(1)
                if not d:
                    continue
                b = d[0]
                if b == FRAME_HEADER and len(buf) == 0:
                    buf.append(b)
                elif len(buf) > 0:
                    buf.append(b)
                    if b == FRAME_TAIL and len(buf) >= FRAME_LEN:
                        self._parse_frame(bytes(buf))
                        buf.clear()
                    elif len(buf) >= FRAME_LEN:
                        buf.clear()
            except Exception as e:
                self.get_logger().warning(f'串口读取错误: {e}')

    def _parse_frame(self, frame):
        """解析STM32底盘速度帧
        帧格式: 0x7B + 0x00 + 0x00 + vx_h + vx_l + vy_h + vy_l + wz_h + wz_l + BCC + 0x7D
        """
        if len(frame) < FRAME_LEN:
            return
        # 校验BCC (字节0-8异或)
        calc_bcc = self._bcc_checksum(frame[0:9])
        if calc_bcc != frame[9]:
            self.get_logger().debug(f'BCC校验失败: calc={calc_bcc:#x} recv={frame[9]:#x}')
            return

        vx_raw = (frame[3] << 8) | frame[4]
        vy_raw = (frame[5] << 8) | frame[6]
        wz_raw = (frame[7] << 8) | frame[8]

        # 有符号转换
        if vx_raw >= 32768:
            vx_raw -= 65536
        if wz_raw >= 32768:
            wz_raw -= 65536

        vx_mps = vx_raw / 1000.0
        wz_rps = wz_raw / 1000.0

        self._update_action_from_velocity(vx_mps, wz_rps)

        self.get_logger().debug(f'收到速度: vx={vx_mps:.3f}m/s wz={wz_rps:.3f}rad/s', throttle_duration_sec=1.0)

    def _update_action_from_velocity(self, vx, wz):
        """根据速度判断动作（与servo_nav2_node一致）"""
        if abs(wz) > TURN_THRESHOLD:
            action = '左转' if wz > 0 else '右转'
        elif abs(vx) > MOVE_THRESHOLD:
            action = '前进' if vx > 0 else '后退'
        else:
            action = '停止'

        now = self.get_clock().now().nanoseconds / 1e9
        if action != self.pending_action:
            self.pending_action = action
            self.pending_since = now

    def _cmd_vel_callback(self, msg):
        """备用：直接接收cmd_vel控制"""
        self._update_action_from_velocity(float(msg.linear.x), float(msg.angular.z))

    def _control_loop(self):
        """20Hz控制循环：0.3s防抖 + 单线程执行步态"""
        now = self.get_clock().now().nanoseconds / 1e9

        if (now - self.pending_since) >= SWITCH_CONFIRM_TIME:
            self.target_action = self.pending_action

        if self.current_action == self.target_action:
            return

        if now - self.last_action_change < ACTION_COOLDOWN:
            return

        with self.thread_lock:
            self.last_action_change = now
            next_action = self.target_action
            self.interrupt_event.set()

            if self.action_thread is not None and self.action_thread.is_alive():
                self.action_thread.join(timeout=1.0)

            self.current_action = next_action
            self.get_logger().info(f'动作切换: {self.current_action}')
            self._execute_action(self.current_action)

    def _execute_action(self, action):
        self.interrupt_event.clear()
        def run():
            if action == '前进':
                self._run_forward_gait()
            elif action == '后退':
                self._run_backward_gait()
            elif action == '左转':
                self._run_left_gait()
            elif action == '右转':
                self._run_right_gait()
            else:
                self._init_pose()
                self.current_vx = 0.0
                self.current_vz = 0.0
        self.action_thread = threading.Thread(target=run, daemon=True)
        self.action_thread.start()

    def bedate(self, num, angle):
        if angle >= 100:
            cmd = f'${num}{angle}#'
        elif angle == 0:
            cmd = f'${num}00{angle}#'
        else:
            cmd = f'${num}0{angle}#'
        return cmd.encode('utf-8')

    def change(self, num, angle):
        if self.ser and self.ser.is_open:
            self.ser.write(self.bedate(num, angle))
            return True
        return False

    def _init_pose(self):
        for num in self.nums1:
            self.change(num, 60)
        for num in self.nums2:
            self.change(num, 90)

    # ==================== 前进步态 ====================
    def _run_forward_gait(self):
        self.current_vx = 0.10
        self.current_vz = -0.11
        steps = [
            (['B', 'H', 'N', 'C', 'I', 'O'], 30),
            (['A', 'G'], 120),
            (['M'], 60),
            (['C', 'I', 'O', 'B', 'H', 'N'], 60),
            (['E', 'K', 'Q', 'F', 'L', 'R'], 30),
            (['A', 'G', 'M'], 90),
            (['D'], 110),
            (['J', 'P'], 60),
            (['F', 'L', 'R', 'E', 'K', 'Q'], 60),
            (['B', 'H', 'N', 'C', 'I', 'O'], 30),
            (['D', 'J', 'P'], 90),
            (['C', 'I', 'O', 'B', 'H', 'N'], 60),
        ]
        while rclpy.ok() and not self.interrupt_event.is_set():
            if self.current_action != '前进':
                break
            for targets, angle in steps:
                if self.interrupt_event.is_set():
                    return
                for num in targets:
                    if self.interrupt_event.is_set():
                        return
                    self.change(num, angle)
                time.sleep(STEP_DURATION)

    # ==================== 后退步态 ====================
    def _run_backward_gait(self):
        self.current_vx = -0.12
        self.current_vz = 0.0
        steps = [
            (['B', 'H', 'N', 'C', 'I', 'O'], 30),
            (['A', 'G'], 60),
            (['M', 'C', 'I', 'O', 'B', 'H', 'N'], 60),
            (['E', 'K', 'Q', 'F', 'L', 'R'], 30),
            (['A', 'G', 'M'], 90),
            (['D'], 60),
            (['J', 'P', 'F', 'L', 'R', 'E', 'K', 'Q'], 60),
            (['B', 'H', 'N', 'C', 'I', 'O'], 30),
            (['D', 'J', 'P'], 90),
            (['C', 'I', 'O', 'B', 'H', 'N'], 60),
        ]
        while rclpy.ok() and not self.interrupt_event.is_set():
            if self.current_action != '后退':
                break
            for targets, angle in steps:
                if self.interrupt_event.is_set():
                    return
                for num in targets:
                    if self.interrupt_event.is_set():
                        return
                    self.change(num, angle)
                time.sleep(STEP_DURATION)

    # ==================== 左转步态 ====================
    def _run_left_gait(self):
        self.current_vx = 0.0
        self.current_vz = 0.11
        steps = [
            (['B', 'H', 'N', 'C', 'I', 'O'], 30),
            (['A', 'G', 'M'], 80),
            (['C', 'I', 'O', 'B', 'H', 'N'], 60),
            (['E', 'K', 'Q', 'F', 'L', 'R'], 30),
            (['A', 'G', 'M'], 90),
            (['D', 'J', 'P'], 80),
            (['F', 'L', 'R', 'E', 'K', 'Q'], 60),
            (['B', 'H', 'N', 'C', 'I', 'O'], 30),
            (['D', 'J', 'P'], 90),
            (['C', 'I', 'O', 'B', 'H', 'N'], 60),
        ]
        while rclpy.ok() and not self.interrupt_event.is_set():
            if self.current_action != '左转':
                break
            for targets, angle in steps:
                if self.interrupt_event.is_set():
                    return
                for num in targets:
                    if self.interrupt_event.is_set():
                        return
                    self.change(num, angle)
                time.sleep(STEP_DURATION)

    # ==================== 右转步态 ====================
    def _run_right_gait(self):
        self.current_vx = 0.0
        self.current_vz = -0.11
        steps = [
            (['B', 'H', 'N', 'C', 'I', 'O'], 30),
            (['A', 'G', 'M'], 95),
            (['C', 'I', 'O', 'B', 'H', 'N'], 60),
            (['E', 'K', 'Q', 'F', 'L', 'R'], 30),
            (['A', 'G', 'M'], 90),
            (['D', 'J', 'P'], 95),
            (['F', 'L', 'R', 'E', 'K', 'Q'], 60),
            (['B', 'H', 'N', 'C', 'I', 'O'], 30),
            (['D', 'J', 'P'], 90),
            (['C', 'I', 'O', 'B', 'H', 'N'], 60),
        ]
        while rclpy.ok() and not self.interrupt_event.is_set():
            if self.current_action != '右转':
                break
            for targets, angle in steps:
                if self.interrupt_event.is_set():
                    return
                for num in targets:
                    if self.interrupt_event.is_set():
                        return
                    self.change(num, angle)
                time.sleep(STEP_DURATION)

    # ==================== 里程计和TF发布 ====================
    def _publish_odom(self):
        from nav_msgs.msg import Odometry
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        if 0.001 < dt < 1.0:
            self.odom_x += self.current_vx * math.cos(self.odom_theta) * dt
            self.odom_y += self.current_vx * math.sin(self.odom_theta) * dt
            self.odom_theta += self.current_vz * dt
            while self.odom_theta > math.pi:
                self.odom_theta -= 2 * math.pi
            while self.odom_theta < -math.pi:
                self.odom_theta += 2 * math.pi

        q = self._euler_to_quaternion(0.0, 0.0, self.odom_theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom_combined'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = float(self.odom_x)
        odom_msg.pose.pose.position.y = float(self.odom_y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = float(self.current_vx)
        odom_msg.twist.twist.angular.z = float(self.current_vz)
        self.odom_pub.publish(odom_msg)

        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom_combined'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = float(self.odom_x)
        t.transform.translation.y = float(self.odom_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x, y, z, w]

    def destroy_node(self):
        self.interrupt_event.set()
        if self.action_thread is not None and self.action_thread.is_alive():
            self.action_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    bridge = Stm32ToChassisBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()