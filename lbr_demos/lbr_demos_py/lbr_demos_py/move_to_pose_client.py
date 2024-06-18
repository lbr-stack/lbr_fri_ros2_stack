import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from lbr_fri_idl.srv import MoveToPose
from lbr_demos_py.asbr import *


class MoveToPoseClient(Node):

    def __init__(self):
        super().__init__('move_to_pose_client')
        self.client = self.create_client(MoveToPose, 'move_to_pose')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = MoveToPose.Request()

    def send_request(self, goal_pose):
        self.request.goal_pose = goal_pose
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = MoveToPoseClient()

    goal_pose = Pose()
    goal_pose.position.x = 0.5
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.5
    # Set orientation as required

    goal_orientation = [180, 10, 180]
    goal_orientation = Rotation.from_ABC(goal_orientation,True)
    goal_orientation = goal_orientation.as_geometry_orientation()
    goal_pose.orientation = goal_orientation 
    
    response = client.send_request(goal_pose)
    print(f'Success: {response.success}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
