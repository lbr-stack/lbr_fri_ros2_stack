import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32
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
        self.curr_pose_subs = self.create_subscription(Pose, 'state/pose', self.curr_pose_update, 1)
        self.curr_pose = Pose()

        self.goal_state = False
        self.commiunication_rate = 0.01

        self.is_init = False

    def send_request(self, goal_pose, lin_vel = 0.005):
        self.request.goal_pose = goal_pose
        self.request.lin_vel = Float32(data = lin_vel)
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

    def curr_pose_update(self, msg):
        self.curr_pose = msg
        self.is_init = True

    def wait_for_init(self):
        while not self.is_init:
            rclpy.spin_once(self, timeout_sec = self.commiunication_rate)
        return 


def main(args=None):
    rclpy.init(args=args)
    client = MoveToPoseClient()
    client.wait_for_init()

    goal_pose = Pose()
    goal_pose.position.x = client.curr_pose.position.x + 0.021
    goal_pose.position.y = client.curr_pose.position.y
    goal_pose.position.z = client.curr_pose.position.z
    # Set orientation as required

    goal_orientation = [180, 0, 180]
    goal_orientation = Rotation.from_ABC(goal_orientation,True)
    goal_orientation = goal_orientation.as_geometry_orientation()
    goal_pose.orientation = goal_orientation 
    
    response = client.send_request(goal_pose, 0.001)
    print(f'Success: {response.success}')
    client.wait_for_goal()
    print('first goal achieved')
    a = input('Press enter to move back')
    # time.sleep(2)

    goal_pose = Pose()
    goal_pose.position.x = client.curr_pose.position.x - 0.035
    goal_pose.position.y = client.curr_pose.position.y
    goal_pose.position.z = client.curr_pose.position.z

    goal_orientation = [180, 0, 180]
    goal_orientation = Rotation.from_ABC(goal_orientation,True)
    goal_orientation = goal_orientation.as_geometry_orientation()
    goal_pose.orientation = goal_orientation 
    
    response = client.send_request(goal_pose, 0.001)
    client.wait_for_goal()
    print('second goal achieved')



    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
