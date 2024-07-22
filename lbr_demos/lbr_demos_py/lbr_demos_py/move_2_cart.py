import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import math
import copy
import time
import numpy as np
from lbr_demos_py.asbr import *
from lbr_fri_idl.srv import MoveToPose, FreeFormMove
import threading

class Move2Cart(Node):

    def __init__(self):
        super().__init__('move_2_cart')
        self.is_init = False
        self.curr_pose = Pose()
        self.goal_pose = Pose()
        self.goal_poses_to_reach = []
        self.lin_vel_in_each_sec = []
        self.lin_vel = False

        self.communication_rate = 0.01  # 10 ms

        self.pose_pub = self.create_publisher(Pose, 'command/pose', 1)
        self.goal_pub = self.create_publisher(Bool, 'goal_reached_top', 1)
        self.pose_sub = self.create_subscription(Pose, 'state/pose', self.on_pose, 1)
        self.service = self.create_service(MoveToPose, 'move_to_pose', self.move_to_pose_callback)
        self.FFM_service = self.create_service(FreeFormMove, 'free_form_move', self.free_form_move_callback)

        self.planning_finished = False
        self.moving_queue = []
        # self.test_text = open('Testing.txt', 'w')
        

    def on_pose(self, msg):
        self.curr_pose = msg
        if not self.is_init:
            self.is_init = True
        if len(self.moving_queue) > 0 and self.planning_finished:
            command_pose = self.moving_queue.pop(0)
            if(self.is_safe_pose(command_pose)):
                # self.test_text.write(self.pose_to_string(command_pose) + '\n')

                self.pose_pub.publish(command_pose)
                # print(command_pose.position)
                if len(self.moving_queue) == 0:
                    goal_reached = Bool()
                    goal_reached.data = True
                    self.goal_pub.publish(goal_reached)
                    # self.test_text.close()
                    # print('Done')

    def pose_to_string(self, pose):
        # Define a custom string representation for the Pose object
        return (f"Position: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}, "
                f"Orientation: x={pose.orientation.x}, y={pose.orientation.y}, "
                f"z={pose.orientation.z}, w={pose.orientation.w}")


    def move_to_pose_callback(self, request, response):
        self.goal_poses_to_reach.append(request.goal_pose)
        self.lin_vel_in_each_sec.append(request.lin_vel.data)

        self.motion_plan()

        response.success = True
        return response

    def free_form_move_callback(self, request, response):
        self.goal_poses_to_reach = request.goal_pose
        self.lin_vel_in_each_sec = [i.data for i in request.lin_vel]
        
        self.motion_plan()

        response.success = True
        return response

    def motion_plan(self):
        self.last_goal_reached = self.curr_pose
        length = len(self.goal_poses_to_reach)
        self.planning_finished = False

        for i_pose in range(length):
            self.goal_pose = self.goal_poses_to_reach.pop(0)
            self.lin_vel = self.lin_vel_in_each_sec.pop(0)
            ang_vel = 0.5 * np.pi / 180 #TODO get from service
            if not self.is_close_pos(self.communication_rate*self.lin_vel):
                self.generate_move_command(self.lin_vel, False)
            elif not self.is_close_orien(self.communication_rate*ang_vel):
                self.generate_move_command(False, ang_vel)
                print('orientation planning only.')

            self.last_goal_reached = self.goal_pose
        self.planning_finished = True


    def generate_move_command(self, lin_vel, ang_vel):  
        
        GoalPose = self.goal_pose
        command_poses = []

        translation_vec = np.asarray([GoalPose.position.x - self.last_goal_reached.position.x,
                                      GoalPose.position.y - self.last_goal_reached.position.y,
                                      GoalPose.position.z - self.last_goal_reached.position.z])
        goal_Rot = Rotation(GoalPose.orientation)
        print(goal_Rot.as_ABC()[0])
        print((Rotation(self.last_goal_reached.orientation)).as_ABC()[0])
        ABC_diff = goal_Rot.as_ABC() - (Rotation(self.last_goal_reached.orientation)).as_ABC()
        for i in range(3):
            if(ABC_diff[i] > np.pi):
                ABC_diff[i] -= 2.0 * np.pi
            elif(ABC_diff[i] < (-np.pi)):
                ABC_diff[i] += 2.0 * np.pi

        print(ABC_diff)

        if(ang_vel != False and lin_vel != False):
            print('ang_vel and lin_vel cannot be both present.')
            return False
        if(ang_vel == False and lin_vel!=False):
            motion_time = np.linalg.norm(translation_vec) / lin_vel
            # vel_vec = (translation_vec / np.linalg.norm(translation_vec)) * lin_vel
        if(ang_vel != False and lin_vel == False):
            motion_time = np.max(np.abs(ABC_diff)) / ang_vel
        

        num_of_steps = int(motion_time / self.communication_rate)

        
        for step_counter in range(num_of_steps):
            command_pose = copy.deepcopy(self.last_goal_reached)
            command_pose.position.x = command_pose.position.x + translation_vec[0] * (step_counter+1) / num_of_steps
            command_pose.position.y = command_pose.position.y + translation_vec[1] * (step_counter+1) / num_of_steps
            command_pose.position.z = command_pose.position.z + translation_vec[2] * (step_counter+1) / num_of_steps
 
            ABC_command = (Rotation(self.last_goal_reached.orientation)).as_ABC() + ABC_diff * (step_counter+1) / num_of_steps
            quat_command = Rotation.from_ABC(ABC_command)
            command_pose.orientation = quat_command.as_geometry_orientation()  

            command_poses.append(command_pose)  

        self.moving_queue.extend(command_poses)
        return 

    def is_close_pos(self, pos_thresh=0.0001):
        if(pos_thresh < 0.00001):
            pos_thresh = 0.00001
        translation_vec = np.asarray([self.goal_pose.position.x - self.last_goal_reached.position.x,
                                      self.goal_pose.position.y - self.last_goal_reached.position.y,
                                      self.goal_pose.position.z - self.last_goal_reached.position.z])

        return np.linalg.norm(translation_vec) < pos_thresh

    def is_close_orien(self, angle_thresh=0.08*np.pi/180.0):
        if(angle_thresh<0.01*np.pi/180.0):
            angle_thresh = 0.01*np.pi/180.0
        last_goal_reached = Rotation(self.last_goal_reached.orientation)
        goal_Rot = Rotation(self.goal_pose.orientation)
        ABC_diff = last_goal_reached.as_ABC() - goal_Rot.as_ABC()
        for i in range(3):
            if(ABC_diff[i] > np.pi):
                ABC_diff[i] -= 2.0 * np.pi
            elif(ABC_diff[i] < (-np.pi)):
                ABC_diff[i] += 2.0 * np.pi

        if(np.max(np.abs(ABC_diff)) < angle_thresh):
            return True

        return False

    def is_safe_pose(self, pose):
        if (pose.position.x > 0.75 or pose.position.x < 0.35 or
            pose.position.y > 0.2 or pose.position.y < -0.2):
            # or
            #pose.position.z > 0.75 or pose.position.z < 0.35):
            print(pose)
            print('Failed, not executable')
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = Move2Cart()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
