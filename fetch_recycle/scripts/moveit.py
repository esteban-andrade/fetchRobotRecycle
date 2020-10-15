#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import sys
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import moveit_commander

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("hi")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    group = moveit_commander.MoveGroupCommander("arm_with_torso")
    group_variable_values = group.get_current_joint_values()
    print ("============ Joint values: "), group_variable_values

    pose = group.get_current_pose()
    print ("============ Joint values: "), pose
    