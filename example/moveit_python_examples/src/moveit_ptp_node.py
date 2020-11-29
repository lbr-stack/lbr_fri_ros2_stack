#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import moveit_commander

# readers might find this doc helpful http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
class PTPCommander(object):
    def __init__(self, group_name='arm'):
        self.group = moveit_commander.MoveGroupCommander(group_name)
        
        self.pub = rospy.Publisher('state/PTP/CartesianPose', PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber('command/PTP/CartesianPose', PoseStamped, callback=self._set_pose_cb, queue_size=1)
        self.sub = rospy.Subscriber('joint_states', JointState, callback=self._get_pose_cb, queue_size=1)

    def _set_pose_cb(self, msg):
        self.group.compute_cartesian_path()

        # return True

    def _get_pose_cb(self, msg):
        pass


if __name__ == '__main__':
    rospy.init_node('moveit_ptp_node')
    ptp_commander = PTPCommander()
    rospy.spin()
