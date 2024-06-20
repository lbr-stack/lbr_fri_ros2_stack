import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import math
import copy
import time
import numpy as np
from lbr_demos_py.asbr import *
from lbr_fri_idl.srv import MoveToPose
import threading

class Move2Cart(Node):

    def __init__(self):
        super().__init__('move_2_cart')
        self.is_init = False
        self.curr_pose = Pose()
        self.goal_pose = Pose()
        self.desired_pose = Pose()

        self.communication_rate = 0.01  # 10 ms
        self.pid_p_correction = 12.2
        self.move_orientation_init_time = -1

        self.pose_pub = self.create_publisher(Pose, 'command/pose', 1)
        self.goal_pub = self.create_publisher(Bool, 'goal_reached_top', 1)
        self.pose_sub = self.create_subscription(Pose, 'state/pose', self.on_pose, 1)
        self.service = self.create_service(MoveToPose, 'move_to_pose', self.move_to_pose_callback)

        self.moving_event = threading.Event()
        self.motion_complete = threading.Condition()

        self.moving_thread = threading.Thread(target=self.move_robot)
        self.moving_thread.start()
        self.lin_vel = False

    def on_pose(self, msg):
        self.curr_pose = msg
        if not self.is_init:
            self.is_init = True
            self.desired_pose = msg
        # print('updating!')

    def move_to_pose_callback(self, request, response):
        self.goal_pose = request.goal_pose
        self.lin_vel = request.lin_vel.data
        self.moving_event.set()


        response.success = True
        return response

    def move_robot(self):
        while rclpy.ok():
            self.moving_event.wait()  # Wait until the event is set
            while not (self.is_close_pos() and self.is_close_orien()):
                if not self.is_close_pos():
                    
                    command_pose = self.generate_move_command(self.lin_vel)
                else:
                    MotionTime = 20.0 #TODO get from service
                    command_pose = self.generate_move_command_rotation(MotionTime)

                if self.is_safe_pose(command_pose) and self.lin_vel<0.1:
                    self.pose_pub.publish(command_pose)
                    print(command_pose.position)
                else:
                    print('Command not safe. Execution halted.')
                    self.moving_event.clear()
                    break

                time.sleep(self.communication_rate)

            self.moving_event.clear()
            temp = Bool()
            temp.data = True
            self.goal_pub.publish(temp)


    def generate_move_command(self, lin_vel, pos_thresh=0.0001):  # Generates move commands, for motions containing ONLY rotational movements use generate_move_command_rotation method
        command_pose = Pose()
        GoalPose = self.goal_pose
        CurrPose = self.curr_pose

        translation_vec = np.asarray([GoalPose.position.x - CurrPose.position.x,
                                      GoalPose.position.y - CurrPose.position.y,
                                      GoalPose.position.z - CurrPose.position.z])
        if(np.linalg.norm(translation_vec) < 2 * pos_thresh):
            vel_vec = (translation_vec / np.linalg.norm(translation_vec)) * lin_vel
        else:
            vel_vec = (translation_vec / np.linalg.norm(translation_vec)) * lin_vel * self.pid_p_correction
        command_pose.position.x = CurrPose.position.x + vel_vec[0] * self.communication_rate
        command_pose.position.y = CurrPose.position.y + vel_vec[1] * self.communication_rate
        command_pose.position.z = CurrPose.position.z + vel_vec[2] * self.communication_rate

        timeRemain2Reach = np.linalg.norm(translation_vec) / lin_vel
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
            self.desired_pose = copy.deepcopy(command_pose)
            self.last_command = command_pose
        else:
            command_pose.orientation = self.last_command.orientation
        # command_pose.orientation = self.initial_pose.orientation

        return command_pose

    def generate_move_command_rotation(self, motion_time, angle_thresh=0.1*3.1415/180.0):
        GoalPose = self.goal_pose
        CurrPose = self.curr_pose
        command_pose = copy.deepcopy(CurrPose)

        if(self.move_orientation_init_time == -1):
            self.move_orientation_init_time = int(round(time.time() * 1000))/1000.0  # Current time in seconds
        else:
            curr_time = int(round(time.time() * 1000))/1000.0  # Current time in seconds
            timeRemain2Reach = motion_time - (curr_time - self.move_orientation_init_time)
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
            self.move_orientation_init_time = -1

        return np.max(np.abs(ABC_diff)) < angle_thresh

    def is_safe_pose(self, pose):
        if (pose.position.x > 0.7 or pose.position.x < 0.4 or
            pose.position.y > 0.1 or pose.position.y < -0.1 or
            pose.position.z > 0.7 or pose.position.z < 0.4):
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
