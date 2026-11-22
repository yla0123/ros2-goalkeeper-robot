import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GoalkeeperControl(Node):

    def __init__(self):
        super().__init__('goalkeeper_control')

        # Current target joint positions
        self._slide_target = 0.0
        self._tilt_target = 0.0

        # Allowed joint ranges
        self._slide_min = -3.55
        self._slide_max = 3.55
        self._tilt_min = -1.0
        self._tilt_max = 1.0

        # Receive desired joint targets
        self.create_subscription(Float64, '/goalkeeper/slide_target', self._slide_callback, 10)

        self.create_subscription(Float64, '/goalkeeper/tilt_target', self._tilt_callback, 10)

        # Send joint trajectory commands
        self._trajectory_publisher = self.create_publisher(JointTrajectory, '/model/goalkeeper_robot/joint_trajectory', 10)

        self.get_logger().info('Goalkeeper control node started')


    def _slide_callback(self, msg):

        self._slide_target = max(self._slide_min, min(msg.data, self._slide_max))
        self._publish_command()


    def _tilt_callback(self, msg):

        self._tilt_target = max(self._tilt_min, min(msg.data, self._tilt_max))
        self._publish_command()


    def _publish_command(self):

        msg = JointTrajectory()
        msg.joint_names = ['slide_joint', 'tilt_joint']

        point = JointTrajectoryPoint()
        point.positions = [self._slide_target, self._tilt_target]

        # Give the controller 1 second to reach the target
        point.time_from_start.sec = 1
        msg.points.append(point)
        self._trajectory_publisher.publish(msg)

        self.get_logger().info(
            f'Command: slide={self._slide_target:.2f} m, '
            f'tilt={self._tilt_target:.2f} rad'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GoalkeeperControl()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()