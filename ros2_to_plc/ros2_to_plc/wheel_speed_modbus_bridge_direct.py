#!/usr/bin/env python3
"""
轮子速度Modbus桥接器 - 直接Modbus通信版本
直接使用pymodbus客户端，不依赖ModbusWrapperClient
自动加载YAML配置文件
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
import yaml
from ament_index_python.packages import get_package_share_directory

try:
    from pymodbus.client import ModbusTcpClient
except ImportError as e:
    raise ImportError("pymodbus 3.x is required. Please install: pip install pymodbus>=3.0.0")


class WheelSpeedModbusBridgeDirect(Node):
    """
    轮子速度Modbus桥接器 - 直接通信版本
    订阅 /wheel_speeds 话题，直接通过Modbus TCP发送到PLC
    """

    def __init__(self):
        super().__init__('wheel_speed_modbus_bridge_direct')

        # 自动加载YAML配置
        self._load_yaml_config()

        # 声明参数（使用YAML中的值作为默认值）
        self.declare_parameter('modbus_ip', self.yaml_config.get('modbus_ip', '192.168.0.1'))
        self.declare_parameter('modbus_port', self.yaml_config.get('modbus_port', 502))
        self.declare_parameter('modbus_unit_id', self.yaml_config.get('modbus_unit_id', 1))
        self.declare_parameter('wheel_speed_topic', self.yaml_config.get('wheel_speed_topic', '/wheel_speeds'))
        self.declare_parameter('modbus_start_address', self.yaml_config.get('modbus_start_address', 0))

        # 获取参数值
        self.modbus_ip = self.get_parameter('modbus_ip').get_parameter_value().string_value
        self.modbus_port = int(self.get_parameter('modbus_port').get_parameter_value().integer_value)
        self.modbus_unit_id = int(self.get_parameter('modbus_unit_id').get_parameter_value().integer_value)
        self.wheel_speed_topic = self.get_parameter('wheel_speed_topic').get_parameter_value().string_value
        self.modbus_start_address = int(self.get_parameter('modbus_start_address').get_parameter_value().integer_value)

        # 初始化Modbus客户端
        try:
            self.modbus_client = ModbusTcpClient(host=self.modbus_ip, port=self.modbus_port)
            if not self.modbus_client.connect():
                raise ConnectionError(f"Failed to connect to {self.modbus_ip}:{self.modbus_port}")
            self.get_logger().info(f'直接Modbus客户端已连接到 {self.modbus_ip}:{self.modbus_port}')
        except Exception as e:
            self.get_logger().error(f'Modbus客户端连接失败: {str(e)}')
            raise

        # 订阅轮子速度话题
        self.subscription = self.create_subscription(
            Float64MultiArray,
            self.wheel_speed_topic,
            self.wheel_speed_callback,
            10
        )

        # 初始化轮子速度数组
        self.wheel_speeds = [0, 0, 0, 0]
        
        # 创建定时器，定期发送速度到PLC
        self.timer = self.create_timer(1.0, self.send_speeds_to_plc)

        self.get_logger().info('轮子速度Modbus桥接器已启动 (直接通信模式)')
        self.get_logger().info(f'订阅话题: {self.wheel_speed_topic}')
        self.get_logger().info(f'Modbus地址: {self.modbus_start_address}-{self.modbus_start_address+3}')
        self.get_logger().info(f'单元ID: {self.modbus_unit_id}')

    def _load_yaml_config(self):
        """自动加载YAML配置文件"""
        self.yaml_config = {}
        
        try:
            # 尝试多个可能的配置文件路径
            possible_paths = [
                # 当前工作目录中的配置文件
                os.path.join(os.getcwd(), 'config', 'wheel_speed_modbus_bridge.yaml'),
                # 包安装目录中的配置文件
                os.path.join(get_package_share_directory('my_modbus_node'), 'config', 'wheel_speed_modbus_bridge.yaml'),
                # 相对路径（开发时）
                os.path.join(os.path.dirname(__file__), '..', 'config', 'wheel_speed_modbus_bridge.yaml'),
            ]
            
            config_file = None
            for path in possible_paths:
                if os.path.exists(path):
                    config_file = path
                    break
            
            if config_file:
                with open(config_file, 'r', encoding='utf-8') as f:
                    yaml_data = yaml.safe_load(f)
                    # 提取 wheel_speed_modbus_bridge_direct 或 wheel_speed_modbus_bridge 的参数
                    if 'wheel_speed_modbus_bridge_direct' in yaml_data:
                        self.yaml_config = yaml_data['wheel_speed_modbus_bridge_direct']['ros__parameters']
                    elif 'wheel_speed_modbus_bridge' in yaml_data:
                        self.yaml_config = yaml_data['wheel_speed_modbus_bridge']['ros__parameters']
                    
                    self.get_logger().info(f'已自动加载配置文件: {config_file}')
                    self.get_logger().debug(f'配置内容: {self.yaml_config}')
            else:
                self.get_logger().warning('未找到配置文件，使用默认参数')
                
        except Exception as e:
            self.get_logger().warning(f'加载配置文件失败: {str(e)}，使用默认参数')

    def wheel_speed_callback(self, msg):
        """
        轮子速度话题回调函数
        """
        if len(msg.data) >= 4:
            # 更新轮子速度数组，将负数转换为无符号16位整数
            self.wheel_speeds = []
            for i in range(4):
                speed = int(float(msg.data[i]))
                # 将负数转换为无符号16位整数表示
                if speed < 0:
                    # 负数转换：-1 -> 65535, -10 -> 65526
                    unsigned_speed = 65536 + speed
                else:
                    unsigned_speed = speed
                self.wheel_speeds.append(unsigned_speed)
            
            self.get_logger().debug(f'接收到轮子速度: 原始={[float(msg.data[i]) for i in range(4)]}, 转换后={self.wheel_speeds}')
        else:
            self.get_logger().warning(f'接收到无效的速度数据，期望4个值，实际收到{len(msg.data)}个')

    def send_speeds_to_plc(self):
        """发送所有轮子速度到PLC - 直接Modbus通信"""
        try:
            # 直接使用pymodbus写入寄存器
            result = self.modbus_client.write_registers(self.modbus_start_address, self.wheel_speeds)
            
            # 检查结果
            if result is None or (hasattr(result, 'isError') and result.isError()):
                self.get_logger().error(f'写入Modbus寄存器失败: {result}')
            else:
                self.get_logger().debug(f'成功写入Modbus寄存器 {self.modbus_start_address}-{self.modbus_start_address+3}')
                self.get_logger().info(f'发送速度: {self.wheel_speeds} (无符号16位整数)')
        except Exception as e:
            self.get_logger().error(f'发送速度到PLC失败: {str(e)}')

    def send_individual_speeds_to_plc(self):
        """分别发送每个轮子的速度到PLC（备用方法）"""
        success_count = 0
        for wheel_index, speed in enumerate(self.wheel_speeds):
            address = self.modbus_start_address + wheel_index
            try:
                result = self.modbus_client.write_register(address, speed)
                if result is None or (hasattr(result, 'isError') and result.isError()):
                    self.get_logger().warning(f'轮子{wheel_index}写入地址{address}失败: {result}')
                else:
                    self.get_logger().debug(f'轮子{wheel_index}速度{speed}已发送到地址{address}')
                    success_count += 1
            except Exception as e:
                self.get_logger().warning(f'轮子{wheel_index}发送失败到地址{address}: {str(e)}')
        
        if success_count > 0:
            self.get_logger().info(f'成功发送{success_count}/4个轮子速度: {self.wheel_speeds}')
        else:
            self.get_logger().error('所有轮子速度发送失败')

    def on_shutdown(self):
        """节点关闭时的清理工作"""
        try:
            if hasattr(self.modbus_client, 'close'):
                self.modbus_client.close()
            self.get_logger().info('Modbus连接已关闭')
        except Exception as e:
            self.get_logger().warning(f'关闭Modbus连接时出错: {e}')


def main():
    rclpy.init()
    node = WheelSpeedModbusBridgeDirect()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
