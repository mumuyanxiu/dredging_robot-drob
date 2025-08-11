#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_pkg_share = FindPackageShare('slam_toolbox')
    pkg_share = get_package_share_directory('nav2_custom_planner')

    # slam_toolbox 参数
    slam_params = PathJoinSubstitution([
        FindPackageShare('nav2_custom_planner'),
        'config',
        'slam_toolbox.yaml'
    ])

    # 启动slam_toolbox（在线模式：持续建图）
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
    )

    # 从TF发布/robot_pose，兼容后续节点
    tf_pose = Node(
        package='nav2_custom_planner',
        executable='tf_pose_publisher',
        name='tf_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'global_frame': 'map', 'base_frame': 'base_footprint', 'publish_rate': 10.0}],
    )

    # 动态路径规划节点（边行走边规划）
    dynamic_planner = Node(
        package='nav2_custom_planner',
        executable='dynamic_planner_node',
        name='dynamic_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 清淤任务管理
    dredging_task = Node(
        package='nav2_custom_planner',
        executable='dredging_task_node',
        name='dredging_task_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_toolbox,
        tf_pose,
        dynamic_planner,
        dredging_task,
    ])
