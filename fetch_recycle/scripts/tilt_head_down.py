#!/usr/bin/env python

import sys
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0, 0.5]

if __name__ == "__main__":
    rospy.init_node("tilt_head_down")

    #Check robot serial number, we never want to run this on a real robot!
    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
        rospy.logerr("This script should not be run on a real robot")
        sys.exit(-1)

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")

    trajectory = JointTrajectory()
    trajectory.joint_names = head_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = head_joint_positions
    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(5.0)

    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory = trajectory
    head_goal.goal_time_tolerance = rospy.Duration(0.0)

    rospy.loginfo("Setting positions...")
    head_client.send_goal(head_goal)

    head_client.wait_for_result(rospy.Duration(6.0))
    rospy.loginfo("...done")