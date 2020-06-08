#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
from tf.transformations import *


if __name__ == "__main__":
    # Initialize moveit_commander and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('d3_move_target_pose', anonymous=False)

    # Get instance from moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Get group_commander from MoveGroupCommander
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)


    # Move using target_pose
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = 0.0
    pose_goal.position.y = -0.6
    pose_goal.position.z = 0.8

    quat = quaternion_from_euler(pi/2.0, -pi/2.0, 0.0)

    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]


    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    quit()