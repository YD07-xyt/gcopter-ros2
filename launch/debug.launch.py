#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gcopter')
    default_params_file = os.path.join(pkg_share, 'config', 'global_planning.yaml')
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the YAML parameter file'
    )
    
    # 获取节点可执行文件的绝对路径
    pkg_prefix = get_package_prefix('gcopter')
    node_executable = os.path.join(pkg_prefix, 'lib', 'gcopter', 'global_planning')
    
    # 检查文件是否存在（若不存在会给出明确错误）
    if not os.path.exists(node_executable):
        raise RuntimeError(f"Node executable not found: {node_executable}")
    
    global_planner_cmd = ExecuteProcess(
        cmd=[
            'gdb', '-ex', 'run', '--args',
            node_executable,
            '--ros-args', '--params-file', LaunchConfiguration('params_file')
        ],
        output='screen',
        emulate_tty=True,
        name='global_planner_node'
    )
    
    return LaunchDescription([
        params_file_arg,
        global_planner_cmd,
    ])