import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import EntityWrench
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity

# import math
import random
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class PenaltyShooter(Node): 
    # inherit from Node

    def __init__(self):
        super().__init__('penalty_shooter')

        self.publisher = self.create_publisher(EntityWrench, '/world/goalkeeper_world/wrench', 10)

        self._delete_client = self.create_client(DeleteEntity, 'world/goalkeeper_world/remove')

        self._spawn_client = self.create_client(SpawnEntity, '/world/goalkeeper_world/create')

        sim_share = Path(get_package_share_directory('goalkeeper_sim'))
        self._ball_model_path = sim_share / 'models' / 'soccer_ball' / 'model.sdf'

        # start the shot after 1 sec
        self._kick_timer = self.create_timer(1.0, self._kick_after_reset) 

        # Initial position
        self._ball_x = -1.0
        self._ball_y = 0.0
        self._ball_z = 0.11

        self._ball_mass = 0.43
        self._goal_x = 10.0

    def kick_ball(self):

        dt = 0.001

        # randomly generate shots
        target_x, target_y, target_z, flight_time = self.generate_shot()
        vx, vy, vz = self.get_launch_velocity(target_x, target_y, target_z, flight_time)

        # Kick strength
        force_x = self._ball_mass * vx / dt
        force_y = self._ball_mass * vy / dt
        force_z = self._ball_mass * vz / dt

        msg = EntityWrench()

        msg.entity.name = 'soccer_ball::ball_link'
        msg.entity.type = Entity.LINK

        msg.wrench.force.x = force_x
        msg.wrench.force.y = force_y
        msg.wrench.force.z = force_z

        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        self.publisher.publish(msg)

        # self.get_logger().info(
        #     f'Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), '
        #     f'flight time: {flight_time:.2f} s'
        # )
        # self.get_logger().info(
        #     f'Launch velocity: ({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s'
        # )

        self.get_logger().info('Ball kicked!')

        self._reset_timer = self.create_timer(2.0, self._reset_after_shot)

    # ============= Delete and Respawn the ball =============
    def reset_ball(self):
        # delete the current ball
        delete_request = DeleteEntity.Request()

        delete_request.entity.name = 'soccer_ball'
        delete_request.entity.type = Entity.MODEL

        # send the delete request
        future = self._delete_client.call_async(delete_request)

        # when object deleted, call _after_delete()
        future.add_done_callback(self._after_delete)

    def _after_delete(self, future):
        try:
            response = future.result()

            if not response.success:
                self.get_logger().error('Failed to delete ball')
                return

            # when ball is deleted successfully, run _spawn_ball() 
            self._spawn_ball()

        except Exception as e:
            self.get_logger().error(f'Delete failed: {e}')

    def _spawn_ball(self):

        # read the model
        with open(self._ball_model_path, 'r') as file:
            sdf_string = file.read()

        spawn_request = SpawnEntity.Request()

        spawn_request.entity_factory.name = 'soccer_ball'
        spawn_request.entity_factory.sdf = sdf_string

        spawn_request.entity_factory.pose.position.x = self._ball_x
        spawn_request.entity_factory.pose.position.y = self._ball_y
        spawn_request.entity_factory.pose.position.z = self._ball_z

        spawn_request.entity_factory.pose.orientation.x = 0.0
        spawn_request.entity_factory.pose.orientation.y = 0.0
        spawn_request.entity_factory.pose.orientation.z = 0.0
        spawn_request.entity_factory.pose.orientation.w = 1.0

        # call _after_spawn after the ball is spawned
        future = self._spawn_client.call_async(spawn_request)
        future.add_done_callback(self._after_spawn)

    def _after_spawn(self, future):
        # send error message if spawn failed
        try:
            response = future.result()

            if response.success:
                self.get_logger().info('Ball reset successfully')

                self._kick_timer = self.create_timer(1.0, self._kick_after_reset)
                # start another kick after 1 sec
            else:
                self.get_logger().error('Failed to spawn ball')

        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')

    def _reset_after_shot(self):
        # this function is called at the end of kick_ball()
        self._reset_timer.cancel()
        self.reset_ball()

    def _kick_after_reset(self):
        # this function is called at the end of reset_ball()
        self._kick_timer.cancel()
        self.kick_ball()

    # ========== Randomly Generate shots ===========
    def get_launch_velocity(self, target_x, target_y, target_z, flight_time):
        g = 9.81

        dx = target_x - self._ball_x
        dy = target_y - self._ball_y
        dz = target_z - self._ball_z

        vx = dx / flight_time
        vy = dy / flight_time
        vz = (dz + 0.5 * g * flight_time**2) / flight_time

        return vx, vy, vz
    
    def generate_shot(self):
        target_x = self._goal_x
        target_y = random.uniform(-3.2, 3.2)
        target_z = random.uniform(0.3, 2.2)
        flight_time = random.uniform(0.5, 0.9)

        return target_x, target_y, target_z, flight_time


def main(args=None):
    rclpy.init(args=args)

    node = PenaltyShooter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()