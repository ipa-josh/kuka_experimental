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

import control_msgs.msg
import trajectory_msgs.msg

def feedback(fb):
	print fb

def test_client():
    client = actionlib.SimpleActionClient('/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()
    client.feedback_cb = feedback

    print "ready to run"

    # Creates a goal to send to the action server.
    goal = control_msgs.msg.FollowJointTrajectoryGoal()

    for i in xrange(6):
        goal.trajectory.joint_names.append("joint"+str(i+1))

    for j in xrange(2):
        pt = trajectory_msgs.msg.JointTrajectoryPoint()
        for i in xrange(len(goal.trajectory.joint_names)):
            if i==2: pt.positions.append(0.001)
            else: pt.positions.append(0)
        goal.trajectory.points.append(pt)

    for i in xrange(6):
        tol = control_msgs.msg.JointTolerance()
        tol.position=0.001
        tol.name="joint"+str(i+1)
        goal.path_tolerance.append(tol)
        goal.goal_tolerance.append(tol)

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
