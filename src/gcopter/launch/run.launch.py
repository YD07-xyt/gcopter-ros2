#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取功能包路径
    pkg_share = get_package_share_directory('gcopter')
    
    # 默认参数文件路径（可选，如果不存在可注释）
    default_params_file = os.path.join(pkg_share, 'config', 'global_planning.yaml')
    
    # 声明启动参数：允许外部传入参数文件路径
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the YAML parameter file'
    )
    
    # 全局规划节点
    global_planner_node = Node(
        package='gcopter',
        executable='global_planning',
        name='global_planning_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        # 也可以直接在 launch 中定义参数（不依赖 yaml），但推荐使用 yaml
        # parameters=[{
        #     'map_topic': '/map',
        #     'target_topic': '/target',
        #     'dilate_radius': 0.2,
        #     ...
        # }]
    )
    
    # 可选：启动 RViz2 可视化（如果配置了相应的 rviz 文件）
    # rviz_config = os.path.join(pkg_share, 'config', 'gcopter.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     output='screen'
    # )
    
    return LaunchDescription([
        params_file_arg,
        global_planner_node,
        # rviz_node,  # 根据需要取消注释
    ])