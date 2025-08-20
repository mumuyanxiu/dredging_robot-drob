#!/usr/bin/env python3
"""
Modbus地址扫描工具
找到PLC支持的可写地址
"""
import rclpy
from rclpy.node import Node

try:
    from pymodbus.client import ModbusTcpClient
except ImportError as e:
    raise ImportError("pymodbus 3.x is required. Please install: pip install pymodbus>=3.0.0")

import time


class AddressScanner(Node):
    """
    地址扫描器
    """

    def __init__(self):
        super().__init__('address_scanner')
        
        # PLC连接参数
        self.plc_ip = '192.168.0.1'
        self.plc_port = 502
        
        # 初始化Modbus客户端
        try:
            self.client = ModbusTcpClient(host=self.plc_ip, port=self.plc_port)
            if not self.client.connect():
                raise ConnectionError(f"Failed to connect to {self.plc_ip}:{self.plc_port}")
            self.get_logger().info(f'已连接到PLC: {self.plc_ip}:{self.plc_port}')
        except Exception as e:
            self.get_logger().error(f'连接PLC失败: {str(e)}')
            raise

    def test_write_address(self, address, value=0):
        """测试单个地址是否可写"""
        try:
            # 尝试写入单个寄存器
            result = self.client.write_register(address, value)
            if result is None or (hasattr(result, 'isError') and result.isError()):
                return False, str(result)
            else:
                return True, "Success"
        except Exception as e:
            return False, str(e)

    def test_write_multiple_addresses(self, start_address, values):
        """测试多个连续地址是否可写"""
        try:
            # 尝试写入多个寄存器
            result = self.client.write_registers(start_address, values)
            if result is None or (hasattr(result, 'isError') and result.isError()):
                return False, str(result)
            else:
                return True, "Success"
        except Exception as e:
            return False, str(e)

    def scan_addresses(self):
        """扫描可写地址"""
        self.get_logger().info('开始扫描可写地址...')
        
        # 测试常用地址范围
        test_ranges = [
            (0, 10, "零基址"),
            (1, 10, "标准基址"),
            (100, 110, "常用范围100-110"),
            (1000, 1010, "高位地址1000-1010"),
            (4001, 4010, "4X寄存器"),
            (40001, 40010, "扩展4X寄存器"),
            (30001, 30010, "输入寄存器范围(测试)"),
        ]
        
        successful_addresses = []
        
        for start, end, description in test_ranges:
            self.get_logger().info(f'\n=== 测试 {description} ({start}-{end}) ===')
            
            range_success = []
            for addr in range(start, end + 1):
                success, message = self.test_write_address(addr, 0)
                if success:
                    self.get_logger().info(f'✅ 地址 {addr}: 可写')
                    range_success.append(addr)
                    successful_addresses.append(addr)
                else:
                    self.get_logger().debug(f'❌ 地址 {addr}: {message}')
                
                time.sleep(0.1)  # 短暂延迟避免过快请求
            
            if range_success:
                self.get_logger().info(f'{description} 可写地址: {range_success}')
        
        if successful_addresses:
            self.get_logger().info(f'\n🎯 总结：所有可写地址: {successful_addresses}')
            
            # 测试连续地址写入
            self.get_logger().info('\n=== 测试连续地址写入 ===')
            for addr in successful_addresses[:5]:  # 测试前5个
                success, message = self.test_write_multiple_addresses(addr, [10, 20, 30, 40])
                if success:
                    self.get_logger().info(f'✅ 连续地址 {addr}-{addr+3}: 可写入多个寄存器')
                    
                    # 尝试写入我们的测试数据
                    test_data = [65526, 10, 65531, 5]  # -10, 10, -5, 5
                    success2, message2 = self.test_write_multiple_addresses(addr, test_data)
                    if success2:
                        self.get_logger().info(f'🎯 推荐地址: {addr} (支持负数转换数据)')
                        return addr
                else:
                    self.get_logger().debug(f'❌ 连续地址 {addr}-{addr+3}: {message}')
        else:
            self.get_logger().error('❌ 未找到任何可写地址！')
            
        return None

    def cleanup(self):
        """清理连接"""
        try:
            self.client.close()
            self.get_logger().info('连接已关闭')
        except:
            pass


def main():
    rclpy.init()
    scanner = AddressScanner()
    
    try:
        recommended_address = scanner.scan_addresses()
        if recommended_address:
            scanner.get_logger().info(f'\n🎯 建议在配置中使用地址: {recommended_address}')
        else:
            scanner.get_logger().error('\n❌ 未找到合适的地址，请检查PLC配置')
    except KeyboardInterrupt:
        pass
    finally:
        scanner.cleanup()
        scanner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
