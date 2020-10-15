#!/usr/bin/env python

import sys

import rospy
import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class JointController:

    def __init__(self):
        rospy.loginfo("Init...")

        rospy.loginfo("Waiting for head_controller...")
        self.head_client = actionlib.SimpleActionClient(
            "head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.head_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for arm_controller...")
        self.arm_client = actionlib.SimpleActionClient(
            "arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")

        self.setHeadTilt()

        arm_intermediate_positions = [1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]
        arm_start_vels = [0, 0, 0, 0, 0, 0, 0]
        self.setArmJoints(arm_intermediate_positions, arm_start_vels)
        arm_joint_positions = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        self.setArmJoints(arm_joint_positions, arm_start_vels)

        self.setGripper(1)

        self.joint_subscriber = rospy.Subscriber(
            "/matlab_joint_config", JointState, self.jointMsgCallback)
        self.gripper_subscriber = rospy.Subscriber(
            "/matlab_gripper_action", Bool, self.gripperMsgCallback)

    def jointMsgCallback(self, msg):
        q = msg.position
        v = msg.velocity
        if not v:
            v = [0, 0, 0, 0, 0, 0, 0]

        self.setArmJoints(q, v)

    def gripperMsgCallback(self, msg):
        self.setGripper(msg.data)

    def setHeadTilt(self):
        head_joint_names = ["head_pan_joint", "head_tilt_joint"]
        head_joint_positions = [0.0, 0.4]

        trajectory = JointTrajectory()
        trajectory.joint_names = head_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = head_joint_positions
        trajectory.points[0].velocities = [0.0 for _ in head_joint_positions]
        trajectory.points[0].accelerations = [
            0.0 for _ in head_joint_positions]
        trajectory.points[0].time_from_start = rospy.Duration(0.0)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = trajectory
        head_goal.goal_time_tolerance = rospy.Duration(0.0)

        rospy.loginfo("Setting Head positions...")
        self.head_client.send_goal(head_goal)
        self.head_client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("...done")

    def setArmJoints(self, joint_states, joint_vel):
        arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                           "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        arm_joint_positions = joint_states
        arm_joint_velocities = joint_vel

        trajectory = JointTrajectory()
        trajectory.joint_names = arm_joint_names

        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = arm_joint_positions
        trajectory.points[0].velocities = arm_joint_velocities
        trajectory.points[0].accelerations = [0.0 for _ in arm_joint_positions]
        trajectory.points[0].time_from_start = rospy.Duration(0.0001)

        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory = trajectory
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)

        rospy.loginfo("Setting Arm positions...")
        self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result()
        rospy.loginfo("...done")

    def setGripper(self, open):
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 10.0
        if open == 1:
            gripper_goal.command.position = 0.1
        else:
            gripper_goal.command.position = 0.0

        rospy.loginfo("Setting Gripper positions...")
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result(rospy.Duration(5.0))
        rospy.loginfo("...done")


if __name__ == "__main__":
    rospy.init_node("joint_control")
    JointController()
    rospy.spin()
