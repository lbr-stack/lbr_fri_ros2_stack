#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


def all_close(goal, actual, tolerance):
    all_equal = True
    for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
            return False

    return True


class CartesianPoseCommander(object):
    def __init__(self):  
        moveit_commander.roscpp_initialize(sys.argv)

        self.group = moveit_commander.MoveGroupCommander('arm')

        self.pub = rospy.Publisher('state/CartesianPose', PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber('command/CartesianPose', PoseStamped, self._set_pose_callback, queue_size=1)
        self.sub = rospy.Subscriber('joint_states', JointState, self._get_pose_callback, queue_size=1)

    def _set_pose_callback(self, pose_stamped_goal):
        b"""Set desired robot pose and execute motion.

            Args:
                pose (geometry_msgs.msg.PoseStamped): Desired pose
            Returns:
                (bool): True if pose reached
        """
        pose_goal = pose_stamped_goal.pose
        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        current_pose = self.group.get_current_pose().pose
        pose_goal = pose_to_list(pose_goal)
        current_pose = pose_to_list(current_pose)
        return all_close(pose_goal, current_pose, 0.01)


    def _get_pose_callback(self, joint_state):
        b"""Get current end-effector pose.

            Returns:
                (bool): True if published
        """
        if not self.pub.publish(self.group.get_current_pose()):
            return False
        return True


if __name__ == '__main__':
    rospy.init_node('cartesian_pose_commander')

    cpc = CartesianPoseCommander()

    rospy.spin()
