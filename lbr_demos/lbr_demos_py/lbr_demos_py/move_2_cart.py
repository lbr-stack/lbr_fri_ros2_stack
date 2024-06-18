#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import copy
import time
import numpy as np
from lbr_demos_py.asbr import * 

class Move2Cart(Node):

    def __init__(self):
        super().__init__('move_2_cart')
        self.is_init = False
        self.curr_pose = Pose()
        self.goal_pose = Pose() # get from constructor?

        self.communication_rate = 0.01  # 10 ms
        self.pid_p_correction = 12.2 # For compensating delay during execution. This value -in accordance with p = 12.2 - was found to be good by manual testings.
                                   # See: https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/187  

        self.move_orientation_init_time = -1

        self.pose_pub = self.create_publisher(Pose, 'command/pose', 1)
        self.pose_sub = self.create_subscription(Pose, 'state/pose', self.on_pose, 1)
        self.flag=1
        

    def on_pose(self, msg):
        
        if not self.is_init:
            self.is_init = True
            self.initial_pose = msg

            #Specify goal pose

            self.goal_pose.position.x = 0.5 
            self.goal_pose.position.y = 0.0
            self.goal_pose.position.z = 0.55

            goal_orientation = Rotation(self.initial_pose.orientation)
            goal_orientation = goal_orientation.as_ABC(True)
            goal_orientation[1]=10.0
            goal_orientation[2]=-160.0
            goal_orientation[0]=180.0
            goal_orientation = Rotation.from_ABC(goal_orientation,True)
            goal_orientation = goal_orientation.as_geometry_orientation()
            self.goal_pose.orientation = goal_orientation 
            
            self.desired_pose = msg # desired_pose: the pose which the robot should be in, at the moment. This is useful particularly in orientation control. 
                                    # For some reason, the orientation gets some noise during execution. Therefore, relying on curr_pose.orientation for generating
                                    # commands was giving problems.

            print('starting at:', int(round(time.time() * 1000)))  # Current time in milliseconds


        else:

            self.curr_pose = msg
            
            if (self.is_close_pos() and self.is_close_orien()):
                
                if(self.flag==1):
                    current_time_ms = int(round(time.time() * 1000))  # Current time in milliseconds
                    print(f'Time (ms): {current_time_ms}')
                    self.flag=0
                return
            elif (self.is_close_pos()):
                MotionTime = 20.0 #s
                command_pose = self.generate_move_command_rotation(MotionTime)
                # print('here')
            else:
                lin_vel = 0.01 # replace the number with desired linear velocity in m/s
                command_pose = self.generate_move_command(lin_vel)
                # print('I AM HERE!')
            

            if (command_pose.position.x > 0.75 or command_pose.position.x < 0.35 or 
                command_pose.position.y > 0.1 or command_pose.position.y < -0.1 or 
                command_pose.position.z > 0.78 or command_pose.position.z < 0.5): #Command guarding
                
                print('Failed, not executable')

            else:
                self.pose_pub.publish(command_pose)
                

    def generate_move_command(self, lin_vel, pos_thresh = 0.0005): #Generates move commands, for motions containing ONLY rotational movements use generate_move_command_rotation method
        command_pose = Pose()
        GoalPose = self.goal_pose
        CurrPose = self.curr_pose

        translation_vec = np.asarray([GoalPose.position.x - CurrPose.position.x, 
                                      GoalPose.position.y - CurrPose.position.y, 
                                      GoalPose.position.z - CurrPose.position.z])
        if(np.linalg.norm(translation_vec)<2*pos_thresh):
            vel_vec = (translation_vec / np.linalg.norm(translation_vec)) * lin_vel 
        else:
            vel_vec = (translation_vec / np.linalg.norm(translation_vec)) * lin_vel * self.pid_p_correction
        command_pose.position.x = CurrPose.position.x + vel_vec[0] * self.communication_rate 
        command_pose.position.y = CurrPose.position.y + vel_vec[1] * self.communication_rate 
        command_pose.position.z = CurrPose.position.z + vel_vec[2] * self.communication_rate 


        timeRemain2Reach = np.linalg.norm(translation_vec) / lin_vel
        if(int(timeRemain2Reach/self.communication_rate)>0):
            curr_Rot = Rotation(CurrPose.orientation)
            goal_Rot = Rotation(GoalPose.orientation)
            ABC_diff = goal_Rot.as_ABC() - (Rotation(self.desired_pose.orientation)).as_ABC()
            for i in range(3):
                if(ABC_diff[i]>np.pi):
                    ABC_diff[i]-=2.0*np.pi
                elif(ABC_diff[i]<(-np.pi)):
                    ABC_diff[i]+=2.0*np.pi

            ABC_step = (ABC_diff / timeRemain2Reach) * self.communication_rate
            ABC_command = ABC_step + (Rotation(self.desired_pose.orientation)).as_ABC()
            quat_command = Rotation.from_ABC(ABC_command)

            command_pose.orientation = quat_command.as_geometry_orientation()
            self.desired_pose = copy.deepcopy(command_pose)
            self.last_command = command_pose
        else:
            command_pose.orientation = self.last_command.orientation
        # command_pose.orientation = self.initial_pose.orientation


        return command_pose

    def generate_move_command_rotation(self, motion_time, angle_thresh = 0.1*3.1415/180.0):
        GoalPose = self.goal_pose
        CurrPose = self.curr_pose
        command_pose = copy.deepcopy(CurrPose)

        if(self.move_orientation_init_time == -1):
            self.move_orientation_init_time = int(round(time.time() * 1000))/1000.0  # Current time in seconds
        else:
            curr_time = int(round(time.time() * 1000))/1000.0  # Current time in seconds
            timeRemain2Reach = motion_time - (curr_time - self.move_orientation_init_time)
            if(int(timeRemain2Reach/self.communication_rate)>0):
                curr_Rot = Rotation(CurrPose.orientation)
                goal_Rot = Rotation(GoalPose.orientation)
                ABC_diff = goal_Rot.as_ABC() - (Rotation(self.desired_pose.orientation)).as_ABC()
                for i in range(3):
                    if(ABC_diff[i]>np.pi):
                        ABC_diff[i]-=2.0*np.pi
                    elif(ABC_diff[i]<(-np.pi)):
                        ABC_diff[i]+=2.0*np.pi

                ABC_step = (ABC_diff / timeRemain2Reach) * self.communication_rate
                ABC_command = ABC_step + (Rotation(self.desired_pose.orientation)).as_ABC()
                quat_command = Rotation.from_ABC(ABC_command)

                command_pose.orientation = quat_command.as_geometry_orientation()
                
                
                # if(np.max(np.abs(ABC_command - curr_Rot.as_ABC()))>0.2*3.14159/180.0):
                #     if(int(round(time.time() * 1000))%5==0):
                #         print(ABC_command)
                #         print(curr_Rot.as_ABC())
                #         print((Rotation(self.desired_pose.orientation)).as_ABC())
                #         print('Too much jump')
                    # return False

                self.last_command = command_pose
                self.desired_pose = copy.deepcopy(command_pose)

            else:
                command_pose.orientation = self.last_command.orientation


        return command_pose



    def is_close_pos(self, pos_thresh = 0.0005):
        translation_vec = np.asarray([self.goal_pose.position.x - self.curr_pose.position.x, 
                                      self.goal_pose.position.y - self.curr_pose.position.y, 
                                      self.goal_pose.position.z - self.curr_pose.position.z])
        
        return np.linalg.norm(translation_vec)<pos_thresh

    def is_close_orien(self, angle_thresh = 0.1*3.1415/180.0):
        curr_Rot = Rotation(self.curr_pose.orientation)
        goal_Rot = Rotation(self.goal_pose.orientation)
        ABC_diff = curr_Rot.as_ABC() - goal_Rot.as_ABC()
        for i in range(3):
            if(ABC_diff[i]>np.pi):
                ABC_diff[i]-=2.0*np.pi
            elif(ABC_diff[i]<(-np.pi)):
                ABC_diff[i]+=2.0*np.pi
        if(np.max(np.abs(ABC_diff))<angle_thresh):
            self.move_orientation_init_time = -1

        return np.max(np.abs(ABC_diff))<angle_thresh


def main(args=None):
    rclpy.init(args=args)
    node = Move2Cart()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()