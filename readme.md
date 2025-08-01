# 四轮差速机器人仿真项目

这是一个基于ROS2的四轮差速机器人仿真项目，实现了机器人的基本控制和导航功能。

## 项目结构

项目包含两个主要的ROS2功能包：

- `drob_control`: 机器人控制包，负责机器人的基本控制功能
- `drob_navigation2`: 导航功能包，实现机器人的自主导航

### 主要功能

- 四轮差速驱动控制
- Gazebo仿真支持
- 激光雷达和IMU传感器支持
- Nav2导航功能
- URDF模型支持

## 依赖项

- ROS2（建议使用Humble或更新版本）
- Gazebo
- Nav2
- ros2_control
- ros2_controllers
- gazebo_ros2_control

## 安装和编译

1. 创建工作空间：
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. 克隆项目：
```bash
git clone <repository_url>
```

3. 安装依赖：
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. 编译项目：
```bash
colcon build
```

5. 设置环境变量：
```bash
source ~/ros2_ws/install/setup.bash
```

## 使用说明

### 启动Gazebo仿真

```bash
ros2 launch drob_control gazebo_sim.launch.py
```

### 启动导航功能

```bash
ros2 launch drob_navigation2 navigation2.launch.py
```

### 手动控制机器人

使用键盘控制：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 配置文件说明

### 控制器配置

控制器配置文件位于 `drob_control/config/controllers.yaml`，包含：
- 差速驱动控制器
- 关节状态广播器
- PID参数配置

### 导航配置

导航参数配置文件位于 `drob_navigation2/config/nav2_params.yaml`，包含：
- 代价地图参数
- 路径规划参数
- 局部导航参数

## 传感器配置

项目支持以下传感器：
- 2D激光雷达
- IMU传感器

## 注意事项

1. 确保所有控制器正确加载：
```bash
ros2 control list_controllers
```

2. 检查TF树是否正确：
```bash
ros2 run tf2_tools view_frames
```

3. 如果遇到odom变换问题，请检查：
   - controllers.yaml中的`open_loop`设置
   - gazebo_control_plugin.xacro中的发布设置

## 常见问题

1. TF变换超时
   - 检查控制器是否正确加载
   - 确保`open_loop`设置为false
   - 检查更新率设置

2. 导航异常
   - 确认传感器数据是否正常发布
   - 检查代价地图参数设置
   - 验证机器人尺寸参数是否正确

## 贡献指南

欢迎提交Issue和Pull Request来改进项目。在提交PR之前，请确保：
- 代码符合ROS2编码规范
- 更新相关文档
- 添加必要的单元测试

## 许可证

本项目采用 [LICENSE] 许可证。

## 联系方式

如有问题，请通过以下方式联系：SV
- 提交Issue
- 发送邮件至 [your-email]