import time
import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Transform, Vector3, PoseWithCovariance, \
    Pose, Point, TwistWithCovariance, Twist
from nav_msgs.msg import Odometry
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE
from odrive.utils import dump_errors
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Header
from tf2_ros import TransformBroadcaster

import odrive
from odrive import enums


def np2nice_string(x):
    return ' '.join([f'{i:+13.3f}' for i in x])


class OdriveCalibration(Node):
    """Calibraties Odrive if necessary."""

    def __init__(self) -> None:
        super().__init__('odrive_calibration')
        self.declare_parameters(namespace='',
                                parameters=[('mode', 'normal'), ])

        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.get_logger().info(f'Running Odrive calibration sequence in {self.mode} mode')
        self.odrv = odrive.find_any(timeout=5)
        self.odrv.clear_errors()
        self.axes = [self.odrv.axis0, self.odrv.axis1, ]
        if self.mode in {'normal', 'verbose'}:
            self.normal_init()
        if self.mode == 'debug':
            self.debug()
        # self.debug()
        self.end_node()

    def destroy_node(self) -> bool:
        return super(OdriveCalibration, self).destroy_node()

    def end_node(self) -> None:
        self.get_logger().info('Odrive calibration sequence finished')
        self.get_logger().info('Shutting down node')
        self.destroy_node()

    def normal_init(self):
        start = time.time()
        init = np.array([axis.encoder.shadow_count for axis in self.axes])
        self.get_logger().info(f'Pre {[axis.current_state for axis in self.axes]}')
        if all(axis.current_state != 1 for axis in self.axes) and self.mode == 'normal':
            self.get_logger().info('Odrive already calibrated')
        else:
            for axis in self.axes:
                axis.requested_state = enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while any(axis.current_state != 1 for axis in self.axes):
                time.sleep(0.05)
                if self.mode == 'verbose':
                    current = init - np.array([axis.encoder.shadow_count for axis in self.axes])
                    relative = current / np.array([axis.encoder.config.cpr for axis in self.axes])
                    debug_info = f"{[axis.current_state for axis in self.axes]} " \
                                 f"{np2nice_string(current)} | {np2nice_string(relative)}"
                    self.get_logger().info(debug_info)
            self.get_logger().info(f'After mid {[axis.current_state for axis in self.axes]}')
        time.sleep(0.2)
        for axis in self.axes:
            axis.requested_state = enums.AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.1)
            axis.controller.config.control_mode = enums.CONTROL_MODE_VELOCITY_CONTROL
        if self.odrv.error:
            self.get_logger().error(f'Odrive error: \n{odrive.utils.dump_errors(self.odrv)}')
        else:
            self.get_logger().info(f'Odrive succesfully setup and in velocity mode. '
                                   f'Took {time.time() - start:4.2f} seconds')
        self.get_logger().info(f'{self.odrv.error=}')
        self.get_logger().info(f'Post {[axis.current_state for axis in self.axes]}')
        odrive.utils.dump_errors(self.odrv, printfunc=self.get_logger().info)

    def debug(self) -> None:
        self.get_logger().info('Odrive DEBUG')
        self.odrv.clear_errors()
        init = np.array([axis.encoder.shadow_count for axis in self.axes])
        try:
            while True:
                time.sleep(0.1)
                current = init - np.array([axis.encoder.shadow_count for axis in self.axes])
                relative = current / np.array([axis.encoder.config.cpr for axis in self.axes])
                debug_info = f"{[axis.current_state for axis in self.axes]} " \
                             f"{np2nice_string(current)} | {np2nice_string(relative)}"
                self.get_logger().info(debug_info)
        except KeyboardInterrupt:
            self.odrv.clear_errors()
            for axis in self.axes:
                axis.requested_state = enums.AXIS_STATE_IDLE
            self.get_logger().info('Exiting and leaving Odrive in IDLE mode')


def main(args=None):  # pragma: no cover
    rclpy.init(args=args)
    node = OdriveCalibration()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':  # pragma: no cover
    main()
