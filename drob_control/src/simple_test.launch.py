#!/usr/bin/env python3

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 获取默认路径
    robot_name_in_model = "drob"
    urdf_tutorial_path = get_package_share_directory('drob_control')
    default_model_path = urdf_tutorial_path + '/urdf/drob/drob.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/test.world'
    
    # 为 Launch 声明参数
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='URDF 的绝对路径')
    
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', LaunchConfiguration('model')]),
        value_type=str)
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        # 传递参数
        launch_arguments=[('world', default_world_path), ('verbose', 'true'), ('gui', 'true')]
    )
    
    # 请求 Gazebo 加载机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model])
    
    # 启动四轮控制器节点
    four_wheel_controller = launch_ros.actions.Node(
        package='drob_control',
        executable='four_wheel_controller',
        name='four_wheel_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wheel_separation': 0.3,
            'wheel_radius': 0.05,
            'max_linear_velocity': 2.0,
            'max_angular_velocity': 3.0,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint'
        }]
    )
    
    return launch.LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        four_wheel_controller,
    ])
