#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import *


if __name__ == "__main__":
    # Initialize moveit_commander and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('d5-1_add_box', anonymous=False)

    # Get instance from moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Get group_commander from MoveGroupCommander
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    rospy.sleep(1.0)


    # Add
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "gripper_eef"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.01 # slightly move from the end effector
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))


    timeout = 4
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        if is_known:
            print "Success"
            quit()

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    print "False"


    quit()