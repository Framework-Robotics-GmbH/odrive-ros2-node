# Copyright 2021 Factor Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

import launch
import launch_ros
import numpy as np
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# from launch_ros.parameters_type import ParameterValue
from launch_ros.parameter_descriptions import ParameterValue


# from rcl_interfaces.msg import ParameterValue


def generate_launch_description():
    declared_arguments = []

    # Find the highest ACM port
    dev_path = Path('/dev/')
    acm_ports = list(dev_path.glob('ttyACM*'))
    default_value = acm_ports[-1] if acm_ports else ''

    params_file = LaunchConfiguration('params_file')
    declare_params_file = launch.actions.DeclareLaunchArgument('params_file')
    declared_arguments.append(declare_params_file)

    declared_arguments.append(
        DeclareLaunchArgument(
            'odrive_port',
            default_value=str(default_value),  # TODO: search for ODrive
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument('init_odrive', default_value='true', )
    )

    to_init_odrive = LaunchConfiguration('init_odrive')  # noqa: F841

    odrive_port = LaunchConfiguration("odrive_port", default=default_value)

    odrive_node = Node(
        name="odrive_node",
        package="odrive_node",
        executable="odrive_node",
        parameters=[
            {
                # 'port': ParameterValue(odrive_port, value_type=str),
                # 'port': str(default_value),
                # 'port': '/dev/ttyAMA0',
                'motor': 2,
                'priority_position_velocity': 1,
                'priority_bus_voltage': 2,
                'priority_temperature': 3,
                'priority_torque': 4,
            },
            params_file,
        ],
        remappings=[
            ('odrive_cmd_velocity', 'cmd_vel'),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        arguments=['--ros-args', '--log-level', 'error', ],
    )

    roboclaw_node = Node(
        name="roboclaw_node",
        package="roboclaw_node",
        executable="roboclaw_node",
        parameters=[
            {
                # 'port': ParameterValue(roboclaw_port, value_type=str),
                # 'port': str(default_value),
                # 'port': '/dev/ttyAMA0',
                'motor': 2,
                'priority_position_velocity': 1,
                'priority_bus_voltage': 2,
                'priority_temperature': 3,
                'priority_torque': 4,
            },
            params_file,
        ],
        remappings=[
            ('roboclaw_cmd_velocity', 'cmd_vel'),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        arguments=['--ros-args', '--log-level', 'info', ],
    )

    odrive_calibration = Node(
        package="odrive_odom",
        executable='init',
        parameters=[
            params_file,
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        # condition=to_init_odrive,
        arguments=['--ros-args', '--log-level', 'warn', ],
    )

    odrive_odom = Node(
        package="odrive_odom",
        executable="odom",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    nodes = [
        # odrive_node,
        roboclaw_node,
        # odrive_odom,
        odrive_calibration,
    ]

    return LaunchDescription(declared_arguments + nodes)
