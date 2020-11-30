#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import math
import copy

# see other examples http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
if __name__ == '__main__':
    rospy.init_node('moveit_motion_examples')

    name = 'arm'
    group = moveit_commander.MoveGroupCommander(name)
    group.set_max_velocity_scaling_factor(0.01)

    # move to a named target
    target = 'home'
    group.set_named_target(target)
    rospy.loginfo('Moving to named target "{}"...'.format(target))
    group.go()
    group.stop()
    rospy.loginfo('Done.')   

    # move to joint position
    current_joint_position = group.get_current_joint_values()
    current_joint_position[1] += math.pi/3.
    current_joint_position[3] -= math.pi/3.
    current_joint_position[5] += math.pi/3.
    rospy.loginfo('Moving to joint goal...')
    group.go(current_joint_position)
    group.stop()
    rospy.loginfo('Done.')

    # point to point motion (PTP)
    pose = group.get_current_pose().pose
    
    waypoints = []

    # move along negative x
    pose.position.x -= 0.1
    waypoints.append(copy.deepcopy(pose))

    # move along y
    pose.position.y += 0.1
    waypoints.append(copy.deepcopy(pose))

    plan, fraction = group.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0.)
    rospy.loginfo('Moving along cartesian path...')
    group.execute(plan)
    group.stop()
    rospy.loginfo('Done.')

    # move to a named target
    target = 'home'
    group.set_named_target(target)
    rospy.loginfo('Moving to named target "{}"...'.format(target))
    group.go()
    group.stop()
    rospy.loginfo('Done.')    

    rospy.spin()
