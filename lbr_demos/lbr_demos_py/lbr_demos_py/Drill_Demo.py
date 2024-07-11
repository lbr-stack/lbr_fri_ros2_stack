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
        super().__init__('Drill_Demo')
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

    def is_close_pos(self, goal_pose, pos_thresh=0.0001):
        if(pos_thresh < 0.00001):
            pos_thresh = 0.00001
        translation_vec = np.asarray([goal_pose.position.x - self.curr_pose.position.x,
                                      goal_pose.position.y - self.curr_pose.position.y,
                                      goal_pose.position.z - self.curr_pose.position.z])

        return np.linalg.norm(translation_vec) < pos_thresh

def main(args=None):
    rclpy.init(args=args)
    client = MoveToPoseClient()
    client.wait_for_init()

    home_pose = Pose()
    home_pose.position.x = 0.55
    home_pose.position.y = 0.0
    home_pose.position.z = 0.50
    home_orientation = [180, 0, 180]
    home_orientation = Rotation.from_ABC(home_orientation,True)
    home_orientation = home_orientation.as_geometry_orientation()
    home_pose.orientation = home_orientation

    goal_pose = Pose()
    goal_pose.position.x = 0.60
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.45
    goal_orientation = [180, 30, 180]
    goal_orientation = Rotation.from_ABC(goal_orientation,True)
    goal_orientation = goal_orientation.as_geometry_orientation()
    goal_pose.orientation = goal_orientation

    






    # print(int(time.time() * 1000))
    if True: #(not client.is_close_pos(home_pose,0.001)):
        response = client.send_request(home_pose, 0.01)
        print(f'Success: {response.success}')
        client.wait_for_goal()
        # print(int(time.time() * 1000))
    print('Home pose achieved.')
    a = input('Press Enter (any key) to continue:')




    
    # print(int(time.time() * 1000))
    response = client.send_request(goal_pose, 0.01)
    # print(f'Success: {response.success}')
    client.wait_for_goal()
    # print(int(time.time() * 1000))
    print('First goal pose achieved.')
    a = input('Press Enter (any key) to move:')
    time.sleep(0.2)

    
    #Insertion
    vector_in_body = np.array([[-0.20], [0], [0], [1]])
    EE_pose = Transformation(client.curr_pose)
    print(EE_pose.m)
    print(EE_pose.m @ vector_in_body)
    final_pose = Pose()
    final_pose.position.x = (EE_pose.m @ vector_in_body)[0][0]
    final_pose.position.y = (EE_pose.m @ vector_in_body)[1][0]
    final_pose.position.z = (EE_pose.m @ vector_in_body)[2][0]

    final_pose.orientation = goal_orientation #Orientation is unchanged

    print(final_pose)
    a = input('Press Enter (any key) to move:')
    time.sleep(0.2)
    # response = client.send_request(final_pose, 0.01)
    # print(f'Success: {response.success}')
    client.wait_for_goal()


    

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
