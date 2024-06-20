import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from lbr_fri_idl.srv import MoveToPose
from lbr_demos_py.asbr import *
import time


class MoveToPoseClient(Node):

    def __init__(self):
        super().__init__('move_to_pose_client')
        self.client = self.create_client(MoveToPose, 'move_to_pose')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = MoveToPose.Request()
        self.subs = self.create_subscription(Bool, 'goal_reached_top', self.goal_reach_update, 1)

        self.goal_state = False
        self.commiunication_rate = 0.01

    def send_request(self, goal_pose, lin_vel = 0.005):
        self.request.goal_pose = goal_pose
        self.request.lin_vel = lin_vel
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def goal_reach_update(self, msg):
        if msg.data == True:
            self.goal_state = True

    def wait_for_goal(self):
        while not self.goal_state:
            rclpy.spin_once(self, timeout_sec = self.commiunication_rate)
        self.goal_state = False
        return 


def main(args=None):
    rclpy.init(args=args)
    client = MoveToPoseClient()

    goal_pose = Pose()
    goal_pose.position.x = 0.512
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.5
    # Set orientation as required

    goal_orientation = [180, 0, 180]
    goal_orientation = Rotation.from_ABC(goal_orientation,True)
    goal_orientation = goal_orientation.as_geometry_orientation()
    goal_pose.orientation = goal_orientation 
    
    response = client.send_request(goal_pose)
    print(f'Success: {response.success}')
    client.wait_for_goal()
    print('first goal achieved')
    # time.sleep(2)

    # goal_pose.position.x = 0.55
    # goal_pose.position.y = 0.0
    # goal_pose.position.z = 0.45
    # # Set orientation as required

    # goal_orientation = [180, 0, 180]
    # goal_orientation = Rotation.from_ABC(goal_orientation,True)
    # goal_orientation = goal_orientation.as_geometry_orientation()
    # goal_pose.orientation = goal_orientation 
    
    # response = client.send_request(goal_pose)
    # client.wait_for_goal()
    # print('second goal achieved')



    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
