import rclpy
import random
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, ApplyLinkWrench
from geometry_msgs.msg import Wrench
from gazebo_msgs.msg import ModelStates
from builtin_interfaces.msg import Duration
import time

class BallSpawner(Node):
    def __init__(self):
        super().__init__('ball_spawner')
        self.get_logger().info('Initializing BallSpawner node...')

        # Clients for spawning, deleting, and applying force to the ball
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, 'delete_entity')
        self.wrench_client = self.create_client(ApplyLinkWrench, '/apply_link_wrench')

        # Wait for the services to become available
        self.wait_for_services()

        # Subscribe to the model states to track the ball's position
        self.position_subscriber = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.position_callback,
            10
        )
        
        self.ball_position = None  # To track the ball’s position for movement verification
        self.get_logger().info('Subscribed to /gazebo/model_states for position tracking.')

        # Start the continuous spawning and throwing process
        self.continuous_spawn_and_throw()

    def wait_for_services(self):
        self.get_logger().info('Waiting for all required services to be available...')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SpawnEntity service...')
        self.get_logger().info('SpawnEntity service is now available.')

        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DeleteEntity service...')
        self.get_logger().info('DeleteEntity service is now available.')

        while not self.wrench_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ApplyLinkWrench service...')
        self.get_logger().info('ApplyLinkWrench service is now available.')

    def position_callback(self, msg):
        # Check if the ball is in the list of models
        if 'ball' in msg.name:
            index = msg.name.index('ball')
            current_position = msg.pose[index].position
            
            # Log initial position or check movement
            if self.ball_position is None:
                self.ball_position = current_position
                self.get_logger().info(f'Initial ball position: ({current_position.x}, {current_position.y}, {current_position.z})')
            else:
                # Check if the ball position has changed significantly
                if abs(current_position.x - self.ball_position.x) < 0.01 and \
                   abs(current_position.y - self.ball_position.y) < 0.01 and \
                   abs(current_position.z - self.ball_position.z) < 0.01:
                    self.get_logger().warn('Ball is not moving significantly; check force application.')
                else:
                    self.get_logger().info(f'Ball moved to: ({current_position.x}, {current_position.y}, {current_position.z})')
                    self.ball_position = current_position  # Update the last known position

    def continuous_spawn_and_throw(self):
        self.get_logger().info('Starting the continuous spawning and throwing process...')
        while rclpy.ok():
            # Delete the existing ball (if it exists)
            self.delete_ball()

            # Delay after deletion
            self.get_logger().info('Waiting 1 second after deleting the ball...')
            time.sleep(1)

            # Spawn and throw the ball
            self.spawn_ball()

            # Wait before applying force
            self.get_logger().info('Waiting 2 seconds after spawning before applying force...')
            time.sleep(2)

            # Apply force to simulate throwing the ball with repeated application
            # self.throw_ball_repeatedly()
            self.throw_ball()

            # Delay to allow the ball to move before the next loop
            self.get_logger().info('Waiting 3 seconds to let the ball move before restarting...')
            time.sleep(3)

    def delete_ball(self):
        self.get_logger().info('Attempting to delete the existing ball...')
        request = DeleteEntity.Request()
        request.name = 'ball'  # Name of the ball entity to delete
        future = self.delete_client.call_async(request)
        future.add_done_callback(self.delete_callback)

    def delete_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Deleted existing ball successfully.')
            else:
                self.get_logger().error(f'Failed to delete ball: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Delete service call failed: {e}')

    def spawn_ball(self):
        self.get_logger().info('Preparing to spawn a new ball...')
        request = SpawnEntity.Request()

        # Generate random spawn position for the ball
        random_x = random.uniform(-8.0, -5.0)  # Spawn far from the wall
        random_y = random.uniform(0.0, 3.0)    # Slightly random y-position
        random_z = random.uniform(1.5, 2.0)    # Hand height (about 1.5m to 2m)

        # Set ball's pose with random coordinates
        request.name = 'ball'
        request.xml = f'''
        <sdf version='1.6'>
            <model name='ball'>
                <pose>{random_x} {random_y} {random_z} 0 0 0</pose>
                <link name='link'>
                    <visual name='visual'>
                        <geometry>
                            <sphere>
                                <radius>0.1</radius>
                            </sphere>
                        </geometry>
                        <material>
                            <ambient>1 0 0 1</ambient>
                        </material>
                    </visual>
                    <collision name='collision'>
                        <geometry>
                            <sphere>
                                <radius>0.1</radius>
                            </sphere>
                        </geometry>
                    </collision>
                </link>
            </model>
        </sdf>
        '''
        request.robot_namespace = 'ball'
        request.reference_frame = 'world'
        self.get_logger().info(f'Spawning ball at position ({random_x}, {random_y}, {random_z})...')
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Ball spawned successfully.')
            else:
                self.get_logger().error(f'Failed to spawn the ball: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Spawn service call failed: {e}')

    def throw_ball_repeatedly(self):
        # Apply a smaller force multiple times to simulate a continuous push
        for i in range():  # Apply force 5 times in short intervals
            self.get_logger().info(f'Applying force burst {i+1}')
            self.throw_ball()
            # time.sleep(0.01)  # Wait half a second between each application

    def throw_ball(self):
        self.get_logger().info('Preparing to apply force to the ball to simulate throwing...')

        # Create a request for the ApplyLinkWrench service
        self.wrench = Wrench()
        self.wrench.force.x = 0.0  # Stronger force in the x direction
        self.wrench.force.y = 8.0 # Increase random y-direction force
        self.wrench.force.z = 8.0  # Stronger upward force for a better throw
        self.wrench.torque.x = 0.0
        self.wrench.torque.y = 0.0
        self.wrench.torque.z = 0.0  # Random spin

        request = ApplyLinkWrench.Request()
        request.link_name = 'ball::link'
        request.wrench = self.wrench
        request.duration.sec = 20000  # Short duration for each burst
        request.duration.nanosec = 0

        self.get_logger().info(
            f'Applying force with values: '
            f'force=({self.wrench.force.x}, {self.wrench.force.y}, {self.wrench.force.z}), '
            f'torque=({self.wrench.torque.x}, {self.wrench.torque.y}, {self.wrench.torque.z}), '
            f'duration={request.duration.sec} seconds.'
        )

        # Send the request asynchronously
        future = self.wrench_client.call_async(request)
        future.add_done_callback(self.force_callback)

    def force_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Force applied to the ball successfully.')
            else:
                self.get_logger().error(f'Failed to apply force: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Apply force service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    ball_spawner = BallSpawner()
    rclpy.spin(ball_spawner)
    ball_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
