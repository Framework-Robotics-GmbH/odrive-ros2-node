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

import numpy as np
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []

    # Find the highest ACM port
    dev_path = Path('/dev/')
    acm_ports = list(dev_path.glob('ttyACM*'))
    default_value = acm_ports[-1] if acm_ports else ''

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

    odrive_port = LaunchConfiguration("odrive_port")

    odrive_node = Node(
        package="odrive_node",
        executable="odrive_node",
        parameters=[
            {'port': odrive_port,
             'motor': 2,
             'priority_position_velocity': 1,
             'priority_bus_voltage': 2,
             'priority_temperature': 3,
             'priority_torque': 4,
             'wheel_dist': 0.26,  # in meters TODO: to a YAML config
             # gear_ratio * circumference * constant[expected time / real time]
             'gear_ratio': (1 / 28.42) * (0.083 * np.pi) * 1.002185502,
             },
        ],
        remappings=[
            ('odrive_cmd_velocity', 'cmd_vel'),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
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

    odrive_calibration = Node(
        package="odrive_odom",
        executable='init',
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        # condition=to_init_odrive,
    )

    nodes = [
        odrive_node,
        odrive_odom,
        odrive_calibration,
    ]

    return LaunchDescription(declared_arguments + nodes)
