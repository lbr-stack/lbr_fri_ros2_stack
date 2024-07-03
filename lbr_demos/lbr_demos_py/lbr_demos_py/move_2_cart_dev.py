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
        self.desired_pose = Pose()
        self.goal_poses_to_reach = []
        self.lin_vel_in_each_sec = []
        self.lin_vel = False
        self.move_init_time = -1

        self.communication_rate = 0.01  # 10 ms

        self.pose_pub = self.create_publisher(Pose, 'command/pose', 1)
        self.goal_pub = self.create_publisher(Bool, 'goal_reached_top', 1)
        self.pose_sub = self.create_subscription(Pose, 'state/pose', self.on_pose, 1)
        self.service = self.create_service(MoveToPose, 'move_to_pose', self.move_to_pose_callback)
        self.service = self.create_service(FreeFormMove, 'free_form_move', self.free_form_move_callback)

        self.planning_finished = False
        self.moving_queue = []
        

    def on_pose(self, msg):
        self.curr_pose = msg
        if not self.is_init:
            self.is_init = True
            self.desired_pose = msg
        if len(self.moveing_queue) > 0 and self.planning_finished:
            command_pose = self.moveing_queue.pop(0)
            self.pose_pub.publish(command_pose)
            print(command_pose.position)
            if len(self.moveing_queue) == 0:
                goal_reached = Bool()
                goal_reached.data = True
                self.goal_pub.publish(goal_reached)

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
            if not self.is_close_pos(self.communication_rate*self.lin_vel):
                
                self.generate_move_command(self.lin_vel)
            elif not self.is_close_orien():
                MotionTime = 20.0 #TODO get from service
                self.generate_move_command_rotation(MotionTime)
            
            self.last_goal_reached = self.goal_pose
        self.planning_finished = True


    def generate_move_command(self, lin_vel):  # Generates move commands, for motions containing ONLY rotational movements use generate_move_command_rotation method
        command_pose = copy.deepcopy(self.last_goal_reached)
        GoalPose = self.goal_pose
        # DesPose = self.desired_pose
        command_poses = []

        translation_vec = np.asarray([GoalPose.position.x - self.last_goal_reached.position.x,
                                      GoalPose.position.y - self.last_goal_reached.position.y,
                                      GoalPose.position.z - self.last_goal_reached.position.z])

        motion_time = np.linalg.norm(translation_vec) / lin_vel
        # vel_vec = (translation_vec / np.linalg.norm(translation_vec)) * lin_vel
        num_of_steps = motion_time / self.communication_rate

        goal_Rot = Rotation(GoalPose.orientation)
        ABC_diff = goal_Rot.as_ABC() - (Rotation(self.last_goal_reached.orientation)).as_ABC()
        for i in range(3):
            if(ABC_diff[i] > np.pi):
                ABC_diff[i] -= 2.0 * np.pi
            elif(ABC_diff[i] < (-np.pi)):
                ABC_diff[i] += 2.0 * np.pi

        for step_counter in range(num_of_steps):
            command_pose.position.x = command_pose.position.x + translation_vec[0] * (step_counter+1) / num_of_steps
            command_pose.position.y = command_pose.position.y + translation_vec[1] * (step_counter+1) / num_of_steps
            command_pose.position.z = command_pose.position.z + translation_vec[2] * (step_counter+1) / num_of_steps
 
            ABC_command = (Rotation(self.last_goal_reached.orientation)).as_ABC() + ABC_diff * (step_counter+1) / num_of_steps
            quat_command = Rotation.from_ABC(ABC_command)
            command_pose.orientation = quat_command.as_geometry_orientation()  

            command_poses.append(command_pose)  

        # self.desired_pose = copy.deepcopy(command_pose)
        self.moving_queue.append(command_poses)
        return 

    def generate_move_command_rotation(self, motion_time, angle_thresh=0.1*3.1415/180.0):
        GoalPose = self.goal_pose
        CurrPose = self.curr_pose
        command_pose = copy.deepcopy(CurrPose)

        if(self.move_init_time == -1):
            self.move_init_time = int(round(time.time() * 1000))/1000.0  # Current time in seconds
        else:
            curr_time = int(round(time.time() * 1000))/1000.0  # Current time in seconds
            timeRemain2Reach = motion_time - (curr_time - self.move_init_time)
            if(int(timeRemain2Reach/self.communication_rate) > 0):
                curr_Rot = Rotation(CurrPose.orientation)
                goal_Rot = Rotation(GoalPose.orientation)
                ABC_diff = goal_Rot.as_ABC() - (Rotation(self.desired_pose.orientation)).as_ABC()
                for i in range(3):
                    if(ABC_diff[i] > np.pi):
                        ABC_diff[i] -= 2.0 * np.pi
                    elif(ABC_diff[i] < (-np.pi)):
                        ABC_diff[i] += 2.0 * np.pi

                ABC_step = (ABC_diff / timeRemain2Reach) * self.communication_rate
                ABC_command = ABC_step + (Rotation(self.desired_pose.orientation)).as_ABC()
                quat_command = Rotation.from_ABC(ABC_command)

                command_pose.orientation = quat_command.as_geometry_orientation()

                self.last_command = command_pose
                self.desired_pose = copy.deepcopy(command_pose)

            else:
                command_pose.orientation = self.last_command.orientation

        return command_pose


        def is_close_pos(self, pos_thresh=0.0001):
            translation_vec = np.asarray([self.goal_pose.position.x - self.curr_pose.position.x,
                                          self.goal_pose.position.y - self.curr_pose.position.y,
                                          self.goal_pose.position.z - self.curr_pose.position.z])

            return np.linalg.norm(translation_vec) < pos_thresh

        def is_close_orien(self, angle_thresh=0.1*3.1415/180.0):
            curr_Rot = Rotation(self.curr_pose.orientation)
            goal_Rot = Rotation(self.goal_pose.orientation)
            ABC_diff = curr_Rot.as_ABC() - goal_Rot.as_ABC()
            for i in range(3):
                if(ABC_diff[i] > np.pi):
                    ABC_diff[i] -= 2.0 * np.pi
                elif(ABC_diff[i] < (-np.pi)):
                    ABC_diff[i] += 2.0 * np.pi
            if(np.max(np.abs(ABC_diff)) < angle_thresh):
                self.move_init_time = -1

            return np.max(np.abs(ABC_diff)) < angle_thresh

def main(args=None):
    rclpy.init(args=args)
    node = Move2Cart()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
