import time
import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Transform, Vector3, PoseWithCovariance, \
    Pose, Point, TwistWithCovariance, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Header
from tf2_ros import TransformBroadcaster

import odrive
from odrive import enums


class OdriveCalibration(Node):
    """Calibraties Odrive if necessary."""

    def __init__(self) -> None:
        super().__init__('odrive_calibration')
        start = time.time()
        logger = self.get_logger()
        logger.info('Running Odrive calibration sequence')
        odrv = odrive.find_any(timeout=5)
        axes = [odrv.axis0, odrv.axis1, ]
        logger.info(f'{[axis.current_state for axis in axes]}')

        if all(axis.current_state != 1 for axis in axes):
            logger.info('Odrive already calibrated')
        else:
            for axis in axes:
                axis.requested_state = enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while any(axis.current_state != 1 for axis in axes):
                time.sleep(0.05)
        for axis in axes:
            axis.requested_state = enums.AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = enums.CONTROL_MODE_VELOCITY_CONTROL
        if odrv.error:
            logger.error(f'Odrive error: \n{odrive.utils.dump_errors(odrv)}')
        else:
            logger.info(f'Odrive succesfully setup and in velocity mode. '
                        f'Took {time.time() - start:4.2f} seconds')


def main(args=None):  # pragma: no cover
    rclpy.init(args=args)
    node = OdriveCalibration()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':  # pragma: no cover
    main()
