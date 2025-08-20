from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_to_plc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'ros2_to_plc'],  # 明确指定包名
    package_dir={package_name: 'ros2_to_plc'},  # 将my_modbus_node映射到ros2_to_plc目录
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='chen',
    maintainer_email='2480665841@qq.com',
    description='ROS2 Modbus节点，支持TCP通信和轮子速度桥接',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_speed_modbus_bridge_direct = ros2_to_plc.wheel_speed_modbus_bridge_direct:main',
            'address_scanner = ros2_to_plc.address_scanner:main',
        ],
    },
)
