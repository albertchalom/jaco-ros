#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco_driver')
import rospy

import actionlib

import jaco_msgs.msg
import geometry_msgs.msg

import sys


def pose_trajectory_client():
    client = actionlib.SimpleActionClient('/jaco/arm_pose_trajectory', jaco_msgs.msg.ArmPoseTrajectoryAction)

    goal = jaco_msgs.msg.ArmPoseTrajectoryGoal()
    goal.trajectory.append(geometry_msgs.msg.PoseStamped())

    goal.trajectory[0].header.frame_id = "/jaco_api_origin"
    
    pose = goal.trajectory[0].pose

    if len(sys.argv) < 8: # default pose
        pose.position.x = -0.15
        pose.position.y = -0.2
        pose.position.z = 0.86

        pose.orientation.x = 0.181744263689
        pose.orientation.y = -0.0420968896875
        pose.orientation.z = 0.00584053291763
        pose.orientation.w = 0.982426975744

        rospy.logwarn("Using test goal: \n%s", goal)
    else: # use pose from command line
        pose.position.x = float(sys.argv[1])
        pose.position.y = float(sys.argv[2])
        pose.position.z = float(sys.argv[3])

        pose.orientation.x = float(sys.argv[4])
        pose.orientation.y = float(sys.argv[5])
        pose.orientation.z = float(sys.argv[6])
        pose.orientation.w = float(sys.argv[7])

    goal.trajectory.append(geometry_msgs.msg.PoseStamped())
    goal.trajectory[1].header.frame_id = "/jaco_api_origin"
    pose = goal.trajectory[1].pose
    pose.position.x = -0.429730504751
    pose.position.y = -0.158013761044
    pose.position.z = 0.387128591537

    pose.orientation.x = -0.298894730133
    pose.orientation.y = -0.576051285325
    pose.orientation.z = 0.268525167981
    pose.orientation.w = 0.711843445664

    client.wait_for_server()
    rospy.loginfo("Connected to Pose server")

    client.send_goal(goal)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('arm_pose_trajectory_client')
        result = pose_trajectory_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
