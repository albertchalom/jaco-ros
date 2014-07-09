#ifndef JACO_ARM_TRAJECTORY_CONTROLLER_H
#define JACO_ARM_TRAJECTORY_CONTROLLER_H

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <boost/thread/recursive_mutex.hpp>
#include <actionlib/server/simple_action_server.h>
#include "jaco_driver/jaco_comm.h"


#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

namespace jaco{

class JacoArmTrajectoryController
{
private:
  ros::Publisher joint_state_pub_;
  ros::Timer joint_state_timer_;

  ros::NodeHandle node_handle_;
  JacoComm &arm_comm_;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajectory_server_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_server_;
  boost::recursive_mutex api_mutex;

public:
  JacoArmTrajectoryController(JacoComm &arm_comm, ros::NodeHandle nh, ros::NodeHandle pnh);
  virtual ~JacoArmTrajectoryController();
  void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
  void execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal);


private:
  std::vector<std::string> joint_names;
  std::vector<double> joint_pos;
  std::vector<double> joint_vel;
  std::vector<double> joint_eff;
  int num_jaco_finger_joints_;
  int num_jaco_joints_;
  unsigned int num_joints_;
  double tolerance_;
  double stall_interval_seconds_;
  double stall_threshold_;
  double rate_hz_;
  ros::Time last_nonstall_time_;
  JacoAngles last_nonstall_angles_;



};

}

#endif // JACO_ARM_TRAJECTORY_CONTROLLER_H
