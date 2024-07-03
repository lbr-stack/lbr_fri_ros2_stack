import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32
from lbr_fri_idl.srv import MoveToPose, FreeFormMove
from lbr_demos_py.asbr import *
import time
import numpy as np

class MoveToPoseClient(Node):

    def __init__(self):
        super().__init__('move_to_pose_client')
        self.client = self.create_client(MoveToPose, 'move_to_pose')
        self.free_form_client = self.create_client(FreeFormMove, 'free_form_move')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = MoveToPose.Request()
        self.FFM_request = FreeFormMove.Request()
        self.subs = self.create_subscription(Bool, 'goal_reached_top', self.goal_reach_update, 1)
        self.curr_pose_subs = self.create_subscription(Pose, 'state/pose', self.curr_pose_update, 1)
        self.curr_pose = Pose()

        self.goal_state = False
        self.commiunication_rate = 0.01

        self.is_init = False

    def send_request(self, goal_pose, lin_vel = 0.005):
        self.request.goal_pose = goal_pose
        self.request.lin_vel = Float32(data = lin_vel)
        print('lin_vel set to ', lin_vel)
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_request_FFM(self, goal_poses, lin_vel):
        self.FFM_request.goal_pose = goal_poses
        if isinstance(lin_vel, list):
            self.FFM_request.lin_vel = [Float32(data=v) for v in lin_vel]
        elif isinstance(lin_vel,float):
            self.FFM_request.lin_vel = [Float32(data=lin_vel)] * len(goal_poses)
        self.future = self.free_form_client.call_async(self.FFM_request)
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
    goal_pose.position.x = 0.55
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.50
    # Set orientation as required

    goal_orientation = [180, 0, 180]
    goal_orientation = Rotation.from_ABC(goal_orientation,True)
    goal_orientation = goal_orientation.as_geometry_orientation()
    goal_pose.orientation = goal_orientation 
    print(int(time.time() * 1000))
    response = client.send_request(goal_pose, 0.01)
    print(f'Success: {response.success}')
    client.wait_for_goal()
    print(int(time.time() * 1000))
    print('first goal achieved')
    a = input('Press enter to move')
    # time.sleep(2)

    # goal_pose = Pose()
    # goal_pose.position.x = client.curr_pose.position.x - 0.035
    # goal_pose.position.y = client.curr_pose.position.y
    # goal_pose.position.z = client.curr_pose.position.z

    # goal_orientation = [180, 0, 180]
    # goal_orientation = Rotation.from_ABC(goal_orientation,True)
    # goal_orientation = goal_orientation.as_geometry_orientation()
    # goal_pose.orientation = goal_orientation 
    
    # response = client.send_request(goal_pose, 0.001)
    # client.wait_for_goal()
    # print('second goal achieved')

    goal_orientation = [180, 0, 180]
    goal_orientation = Rotation.from_ABC(goal_orientation,True)
    goal_orientation = goal_orientation.as_geometry_orientation()

    pitch = 0.0
    radius = 0.05
    turns = 0.25
    resolution = 1.2 * np.pi / 180.0
    num_of_steps = turns * 2 * np.pi / resolution
    poses=[] 
    print(num_of_steps)
    a = input('Press enter to move')
    for i in range(int(num_of_steps)):
        goal_pose = Pose()
        goal_pose.orientation = goal_orientation
        goal_pose.position.x = client.curr_pose.position.x + (1-np.cos(resolution*i))*radius
        goal_pose.position.y = client.curr_pose.position.y + (np.sin(resolution*i))*radius
        goal_pose.position.z = client.curr_pose.position.z + i*pitch*resolution/(2*np.pi)
        poses.append(goal_pose)




    # goal_pose_1 = Pose()
    # goal_pose_1.position.x = client.curr_pose.position.x + 0.021
    # goal_pose_1.position.y = client.curr_pose.position.y
    # goal_pose_1.position.z = client.curr_pose.position.z
    # # Set orientation as required

    
    # goal_pose_1.orientation = goal_orientation 
    
    # goal_pose_2 = Pose()
    # goal_pose_2.position.x = client.curr_pose.position.x - 0.021
    # goal_pose_2.position.y = client.curr_pose.position.y
    # goal_pose_2.position.z = client.curr_pose.position.z 
    # goal_pose_2.orientation = goal_orientation

    # goal_pose_3 = Pose()
    # goal_pose_3.position.x = client.curr_pose.position.x 
    # goal_pose_3.position.y = client.curr_pose.position.y - 0.1
    # goal_pose_3.position.z = client.curr_pose.position.z - 0.05
    # goal_pose_3.orientation = goal_orientation

    # goal_poses = [goal_pose_1, goal_pose_2]# , goal_pose_3]

    current_time_milliseconds = int(time.time() * 1000)

    # Print the current time in milliseconds
    print(f"Current time in milliseconds: {current_time_milliseconds}")

    # response = client.send_request_FFM(poses, 0.005)
    print(f'Success: {response.success}')
    client.wait_for_goal()
    print('All goals achieved')

    current_time_milliseconds = int(time.time() * 1000)

    # Print the current time in milliseconds
    print(f"Current time in milliseconds: {current_time_milliseconds}")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
