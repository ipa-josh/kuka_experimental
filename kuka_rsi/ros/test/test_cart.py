#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2014 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n
#
#   All rights reserved. \n\n
#
#################################################################
#
# \note
#   ROS package name: ipa_kuka_rsi
#
# \author
#   Author: Joshua Hampp
#
# \date Date of creation: 06/21/2014
#
# \brief
#   test script
#
#################################################################

import roslib; roslib.load_manifest('ipa_kuka_rsi')
import rospy

# Brings in the SimpleActionClient
import actionlib

import moveit_msgs.msg
import trajectory_msgs.msg

def feedback(fb):
	print fb

def test_client():
    client = actionlib.SimpleActionClient('/move_cartesian', moveit_msgs.msg.MoveGroupAction)
    client.wait_for_server()
    client.feedback_cb = feedback

    print "ready to run"

    # Creates a goal to send to the action server.
    goal = moveit_msgs.msg.MoveGroupGoal()

    const = moveit_msgs.msg.Constraints()
    pos = moveit_msgs.msg.PositionConstraint()
    orient = moveit_msgs.msg.OrientationConstraint()
    pos.link_name="tip"
    orient.link_name="tip"
    pos.header.frame_id="base"
    orient.header.frame_id="base"
    pos.target_point_offset.x = 0.003
    pos.target_point_offset.y = 0.001
    pos.target_point_offset.z = 0.0001
    orient.orientation.x = 0
    orient.orientation.y = 0
    orient.orientation.z = 0
    orient.orientation.w = 1
    orient.weight = 0.005
    pos.weight = 0.0001
    const.position_constraints.append(pos)
    const.orientation_constraints.append(orient)
    goal.request.goal_constraints.append(const)
    #goal.header.frame_id="base" # --> no transformation needed

    #print goal
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('test_client_py')
        result = test_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
