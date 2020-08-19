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

from udm_hand_control.srv import *

def handle_req(req):
    moveit_commander.roscpp_initialize(sys.argv)
    print req.orientation
    print req.posex
    print req.posey
    print req.posez
    print req.group

    group_name = req.group

    print req.group.data
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()


    move_group = moveit_commander.MoveGroupCommander(req.group.data)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = req.orientation.data
    pose_goal.position.x = req.posex.data
    pose_goal.position.y = req.posey.data
    pose_goal.position.z = req.posez.data

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rep=udm_serviceResponse()

    rep.res=True
    return rep

def simple_server():
    rospy.init_node('move_hand', anonymous=True)
    s = rospy.Service('cinematic_service', cinematic_service, handle_req)
    rospy.spin()


if __name__ == '__main__':
  #main()
  simple_server()




