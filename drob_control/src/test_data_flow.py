#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time

class DataFlowTester(Node):
    def __init__(self):
        super().__init__('data_flow_tester')
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 创建订阅者来监控控制器命令
        self.lf_cmd_sub = self.create_subscription(
            Float64MultiArray, '/drob_left_front_velocity_controller/commands', 
            self.lf_cmd_callback, 10)
        self.rf_cmd_sub = self.create_subscription(
            Float64MultiArray, '/drob_right_front_velocity_controller/commands', 
            self.rf_cmd_callback, 10)
        
        self.lf_cmd_received = False
        self.rf_cmd_received = False
        
        self.get_logger().info('数据流测试器启动')
        
    def lf_cmd_callback(self, msg):
        if msg.data:
            self.get_logger().info(f'✓ 左前轮命令: {msg.data[0]:.6f} rad/s')
            self.lf_cmd_received = True
    
    def rf_cmd_callback(self, msg):
        if msg.data:
            self.get_logger().info(f'✓ 右前轮命令: {msg.data[0]:.6f} rad/s')
            self.rf_cmd_received = True
    
    def send_test_command(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'发送cmd_vel: 线速度={linear_x:.2f}, 角速度={angular_z:.2f}')
    
    def test_data_flow(self):
        self.get_logger().info('开始数据流测试...')
        
        # 等待系统初始化
        time.sleep(2)
        
        # 测试1: 发送前进命令
        self.get_logger().info('=== 测试1: 发送前进命令 ===')
        self.send_test_command(linear_x=1.0, angular_z=0.0)
        time.sleep(1)
        
        if not (self.lf_cmd_received and self.rf_cmd_received):
            self.get_logger().error('❌ 控制器命令话题没有收到数据！')
        else:
            self.get_logger().info('✅ 控制器命令话题收到数据')
        
        # 测试2: 发送停止命令
        self.get_logger().info('=== 测试2: 发送停止命令 ===')
        self.send_test_command(linear_x=0.0, angular_z=0.0)
        time.sleep(1)
        
        # 测试3: 发送转向命令
        self.get_logger().info('=== 测试3: 发送转向命令 ===')
        self.send_test_command(linear_x=0.0, angular_z=1.0)
        time.sleep(1)
        
        self.get_logger().info('数据流测试完成')

def main(args=None):
    rclpy.init(args=args)
    tester = DataFlowTester()
    
    try:
        tester.test_data_flow()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 