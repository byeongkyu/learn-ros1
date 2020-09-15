#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from tf.transformations import *
from moveit_msgs.msg import RobotState, Constraints, JointConstraint


if __name__ == "__main__":
    # Initialize moveit_commander and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('d6_move_target_pose_w_constraint', anonymous=False)

    # Get instance from moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Get group_commander from MoveGroupCommander
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)


    # Move using target_pose with constraint
    contraints = Constraints()

    contraints.name = "constraints1"

    jc1 = JointConstraint()
    jc1.joint_name = "ur5e_wrist_3_joint"
    jc1.position = 0.0
    jc1.tolerance_above = 0.01
    jc1.tolerance_below = 0.01

    contraints.joint_constraints.append(jc1)
    move_group.set_path_constraints(contraints)


    # Move
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = 0.525
    pose_goal.position.y =  -0.5
    pose_goal.position.z = 1.0

    quat = quaternion_from_euler(pi/2.0, -pi/2.0, pi/2.0)

    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]

    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    move_group.set_path_constraints(None)

    quit()