"""Based on Nav2 and ROS diff drive odometry tutorial."""
import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Transform, Vector3, \
    PoseWithCovariance, Pose, Point, TwistWithCovariance, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class OdriveOdom(Node):
    """Computes odometry from Odrive position."""

    def __init__(self) -> None:
        super().__init__('odrive_odometry')

        self.odrive_twist_sub = self.create_subscription(TwistStamped, '/odrive_odom_velocity',
                                                         self.process_twist_odom, 10)
        self.nav2_odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_frame_id = 'odom'
        self.child_frame_id = 'base_link'

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now().to_msg()

        self.get_logger().info('Initialized')

    def process_twist_odom(self, twist_stamped_odom: TwistStamped):
        """Callback for when the odrive position is ready."""

        current_time = twist_stamped_odom.header.stamp

        dt = (
                    current_time.sec + current_time.nanosec / 1e9 - self.last_time.sec - self.last_time.nanosec / 1e9)
        twist_odom = twist_stamped_odom.twist
        vx = twist_odom.linear.x
        vth = twist_odom.angular.z
        delta_x = vx * np.cos(vth) * dt
        delta_y = vx * np.sin(vth) * dt
        delta_th = vth * dt * 2.2

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = R.from_euler('z', self.th).as_quat()
        header = Header(stamp=current_time, frame_id=self.odom_frame_id)
        quaternion_msg = Quaternion(x=odom_quat[0], y=odom_quat[1], z=odom_quat[2], w=odom_quat[3])
        odom_trans = TransformStamped(header=header,
                                      child_frame_id=self.child_frame_id,
                                      transform=Transform(
                                          translation=Vector3(x=self.x, y=self.y, z=0.0),
                                          rotation=quaternion_msg))
        self.tf_broadcaster.sendTransform(odom_trans)

        odom = Odometry(header=header,
                        child_frame_id=self.child_frame_id,
                        pose=PoseWithCovariance(
                            pose=Pose(
                                position=Point(x=self.x, y=self.y, z=0.0),
                                orientation=quaternion_msg),
                        ),
                        twist=TwistWithCovariance(
                            twist=twist_odom
                        )
                        )
        self.nav2_odom_pub.publish(odom)

        self.last_time = current_time


def main(args=None):  # pragma: no cover
    rclpy.init(args=args)
    node = OdriveOdom()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':  # pragma: no cover
    main()
