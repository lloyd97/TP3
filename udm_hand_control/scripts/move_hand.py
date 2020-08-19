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
    print req.joint_goal1
    print req.joint_goal2
    print req.joint_goal3
    print req.group

    group_name = req.group

    print req.group.data
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    if(req.group.data == "thumb"):
       move_group = moveit_commander.MoveGroupCommander(req.group.data)
       display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = req.joint_goal1.data
       joint_goal[1] = req.joint_goal2.data
       move_group.go(joint_goal, wait=True)
       move_group.stop()
       current_joints = move_group.get_current_joint_values()

    else: 
       move_group = moveit_commander.MoveGroupCommander(req.group.data)
       display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = req.joint_goal1.data
       joint_goal[1] = req.joint_goal2.data
       joint_goal[2] = req.joint_goal2.data
       move_group.go(joint_goal, wait=True)
       move_group.stop()
       current_joints = move_group.get_current_joint_values()
    
    rep=udm_serviceResponse()

    rep.res=True
    return rep

def simple_server():
    rospy.init_node('move_hand', anonymous=True)
    s = rospy.Service('udm_service', udm_service, handle_req)
    rospy.spin()


if __name__ == '__main__':
  #main()
  simple_server()




