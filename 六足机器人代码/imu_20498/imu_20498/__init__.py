#!/usr/bin/env python3
"""
IMU20498 串口驱动 - USB1端口
通过 UART 读取 9 轴 IMU 数据，发布 /imu/data_raw
"""

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import struct
import threading
import time

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200


class IMU20498Node(Node):
    def __init__(self):
        super().__init__('imu_20498_driver')
        
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUD)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100)  # 100Hz
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value
        
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=1.0
            )
            self.get_logger().info(f'Opened serial port {self.port} at {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return
        
        self.running = True
        self.recv_thread = threading.Thread(target=self.recv_loop)
        self.recv_thread.daemon = True
        self.recv_thread.start()
        
        # IMU 数据
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        period = 1.0 / rate
        self.timer = self.create_timer(period, self.publish_imu)
        
        self.get_logger().info('IMU20498 driver started')
    
    def recv_loop(self):
        buffer = b''
        
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer += data
                    
                    # 查找 0x7E 帧头
                    while len(buffer) >= 11:
                        if buffer[0] == 0x7E:
                            self.parse_frame(buffer[:11])
                            buffer = buffer[11:]
                        else:
                            buffer = buffer[1:]
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.get_logger().error(f'Recv error: {e}')
                self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        time.sleep(0.1)
    
    def parse_frame(self, frame):
        """解析 IMU20498 数据帧"""
        if len(frame) < 11:
            return
        
        data_type = frame[1]
        
        # 加速度计 (0x23)
        if data_type == 0x23:
            try:
                ax = struct.unpack('<h', frame[2:4])[0]
                ay = struct.unpack('<h', frame[4:6])[0]
                az = struct.unpack('<h', frame[6:8])[0]
                # 量程: ±16g
                self.acc = [
                    ax / 32768.0 * 16.0 * 9.8,
                    ay / 32768.0 * 16.0 * 9.8,
                    az / 32768.0 * 16.0 * 9.8
                ]
            except:
                pass
        
        # 陀螺仪 (0x25)
        elif data_type == 0x25:
            try:
                gx = struct.unpack('<h', frame[2:4])[0]
                gy = struct.unpack('<h', frame[4:6])[0]
                gz = struct.unpack('<h', frame[6:8])[0]
                # 量程: ±2000°/s
                self.gyro = [
                    gx / 32768.0 * 2000.0 * math.pi / 180,
                    gy / 32768.0 * 2000.0 * math.pi / 180,
                    gz / 32768.0 * 2000.0 * math.pi / 180
                ]
            except:
                pass
    
    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]
        
        msg.angular_velocity.x = self.gyro[0]
        msg.angular_velocity.y = self.gyro[1]
        msg.angular_velocity.z = self.gyro[2]
        
        # 协方差矩阵
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        self.imu_pub.publish(msg)
    
    def close(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.get_logger().info('Serial port closed')


def main(args=None):
    rclpy.init(args=args)
    node = IMU20498Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'close'):
            node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
