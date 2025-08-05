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
    
    # 加载并激活 drob_joint_state_broadcaster 控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'drob_joint_state_broadcaster'],
        output='screen'
    )

    # 加载并激活四轮速度控制器
    load_left_front_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'drob_left_front_velocity_controller'], 
        output='screen')
    
    load_right_front_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'drob_right_front_velocity_controller'], 
        output='screen')
    
    load_left_back_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'drob_left_back_velocity_controller'], 
        output='screen')
    
    load_right_back_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'drob_right_back_velocity_controller'], 
        output='screen')
    

    
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
        # 事件动作，当加载机器人结束后执行    
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],)
            ),
        # 事件动作，当joint_state_broadcaster加载完成后执行
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_left_front_controller],)
            ),
        # 事件动作，当左前轮控制器加载完成后执行
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_left_front_controller,
                on_exit=[load_right_front_controller],)
            ),
        # 事件动作，当右前轮控制器加载完成后执行
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_right_front_controller,
                on_exit=[load_left_back_controller],)
            ),
        # 事件动作，当左后轮控制器加载完成后执行
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_left_back_controller,
                on_exit=[load_right_back_controller],)
            ),
        # 事件动作，当右后轮控制器加载完成后执行
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_right_back_controller,
                on_exit=[four_wheel_controller],)
            ),
    ]) 