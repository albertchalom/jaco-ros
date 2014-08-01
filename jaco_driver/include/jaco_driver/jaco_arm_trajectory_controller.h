#ifndef JACO_ARM_TRAJECTORY_NODE_H_
#define JACO_ARM_TRAJECTORY_NODE_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <ecl/geometry.hpp>
#include <geometry_msgs/Twist.h>
#include <jaco_msgs/ExecuteGraspAction.h>
#include <jaco_msgs/EulerToQuaternion.h>
#include <jaco_msgs/JacoFingerVel.h>
#include <jaco_msgs/JacoFK.h>
#include <jaco_msgs/QuaternionToEuler.h>
#include "jaco_driver/jaco_comm.h"
#include <math.h>

#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3
#define NUM_JOINTS (NUM_JACO_JOINTS+NUM_JACO_FINGER_JOINTS)

#define LARGE_ACTUATOR_VELOCITY 0.8378 //maximum velocity of large actuator (joints 1-3)
#define SMALL_ACTUATOR_VELOCITY 1.0472 //maximum velocity of small actuator (joints 4-6)
#define TIME_SCALING_FACTOR 4 //keep the trajectory at a followable speed

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

#define MAX_FINGER_VEL 30 //maximum finger actuator velocity

//gains for trajectory follower
#define KP 300.0
#define KV 20.0
#define ERROR_THRESHOLD .03 //threshold in radians for combined joint error to consider motion a success

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
  JacoArmTrajectoryController(JacoComm &arm_comm, ros::NodeHandle nh);
  virtual ~JacoArmTrajectoryController();
  void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
  void execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal);
  void build_trajectory_spline(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
  void execute_joint_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

private:
  std::vector<std::string> joint_names;
  std::vector<double> joint_pos;
  std::vector<double> joint_vel;
  std::vector<double> joint_eff;  
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smooth_joint_trajectory_server;
  // Spline vector populated by build_trajectory_spline and execute_joint_trajectory
  
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
