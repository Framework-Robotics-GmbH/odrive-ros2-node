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
import time
from time import sleep

import numpy as np
import odrive
from odrive import enums
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    declared_arguments = []

    # Find the highest ACM port
    dev_path = Path('/dev/')
    acm_ports = list(dev_path.glob('ttyACM*'))
    default_value = acm_ports[-1] if acm_ports else ''

    declared_arguments.append(
        DeclareLaunchArgument(
            'odrive_port',
            default_value=str(default_value),   # TODO: search for ODrive
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

#
# def init_odrive():
#     start = time.time()
#     logger = get_logger('launch')
#     logger.info('Running Odrive calibration sequence')
#     odrv = odrive.find_any(timeout=5)
#     axes = [odrv.axis0, odrv.axis1, ]
#     logger.info(f'{[axis.current_state for axis in axes]}')
#     if all(axis.current_state != 1 for axis in axes):
#         logger.info('Odrive already calibrated')
#     else:
#         for axis in axes:
#             axis.requested_state = enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
#         while any(axis.current_state != 1 for axis in axes):
#             sleep(0.05)
#     for axis in axes:
#         axis.requested_state = enums.AXIS_STATE_CLOSED_LOOP_CONTROL
#         axis.controller.config.control_mode = enums.CONTROL_MODE_VELOCITY_CONTROL
#     if odrv.error:
#         logger.error(f'Odrive error: \n{odrive.utils.dump_errors(odrv)}')
#     else:
#         logger.info(f'Odrive succesfully setup and in velocity mode. '
#                     f'Took {time.time() - start:4.2f} seconds')
