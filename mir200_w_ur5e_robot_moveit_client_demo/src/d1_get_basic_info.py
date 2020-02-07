#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


if __name__ == "__main__":
    # Initialize moveit_commander and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('d1_get_basic_info', anonymous=False)

    # Get instance from moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Get group_commander from MoveGroupCommander
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)


    # Get information
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()
    current_state = robot.get_current_state()

    print "============ Planning frame: %s" % planning_frame
    print "============ End effector link: %s" % eef_link
    print "============ Available Planning Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print current_state
    print "="*20

    quit()

