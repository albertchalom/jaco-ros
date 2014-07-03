
#include <jaco_driver/jaco_arm_trajectory_controller.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <boost/foreach.hpp>
#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"


namespace jaco_arm{

  JacoArmTrajectoryController::JacoArmTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh) :
      arm_comm_(arm_comm),
      node_handle_(nh, "joint_angles"),
      trajectory_server_(nh, "arm_controller", boost::bind(&JacoArmTrajectoryController::execute_trajectory, this, _1), false),
      gripper_server_(nh, "fingers_controller", boost::bind(&JacoArmTrajectoryController::execute_gripper, this, _1), false)
  {

      node_handle_.param<unsigned int>("num_jaco_finger_joints", num_jaco_finger_joints_, NUM_JACO_FINGER_JOINTS);
      node_handle_.param<unsigned int>("num_jaco_joints", num_jaco_joints_, NUM_JACO_JOINTS);
      node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
      node_handle_.param<double>("stall_threshold", stall_threshold_, 1.0);
      node_handle_.param<double>("rate_hz", rate_hz_, 10.0);

      double tolerance;
      node_handle_.param<double>("tolerance", tolerance, 2.0);
      tolerance_ = (float)tolerance;
      num_joints_ = num_jaco_joints_ + num_jaco_finger_joints_;

    for(int joint_id = 0; joint_id < num_jaco_joints_; ++joint_id){
      std::stringstream joint_name_stream;
      joint_name_stream << "jaco_joint_" << (joint_id+1);
      std::string joint_name = joint_name_stream.str();
      joint_names.push_back(joint_name);
    }
    for(int finger_id = 0; finger_id < num_jaco_finger_joints_; ++finger_id){
      std::stringstream finger_name_stream;
      finger_name_stream << "jaco_joint_finger_" << (finger_id+1);
      std::string finger_name = finger_name_stream.str();
      joint_names.push_back(finger_name);
    }

    joint_pos.resize(joint_num);
    joint_vel.resize(joint_num);
    joint_eff.resize(joint_num);

    trajectory_server_.start();
    gripper_server_.start();

    joint_state_timer_ = nh.createTimer(ros::Duration(0.05), boost::bind(&JacoArmTrajectoryController::update_joint_states, this));
  }

  JacoArmTrajectoryController::~JacoArmTrajectoryController()
  {
  }


  static inline double nearest_equivelent(double desired, double current){
    double previous_rev = floor(current / (2*M_PI));
    double next_rev = ceil(current / (2*M_PI));
    double lower_desired = previous_rev*(2*M_PI) + desired;
    double upper_desired = next_rev*(2*M_PI) + desired;
    if(abs(current - lower_desired) < abs(current - upper_desired))
      return lower_desired;
    return upper_desired;
  }

  void JacoArmTrajectoryController::execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal){
    {
      control_msgs::FollowJointTrajectoryFeedback feedback;
      control_msgs::FollowJointTrajectoryResult result;
      JacoAngles current_joint_angles;
      ros::Time current_time = ros::Time::now();


      try
      {
          arm_comm_.getJointAngles(current_joint_angles);

          if (arm_comm_.isStopped())
          {
              ROS_ERROR("Could not complete joint angle action because the arm is 'stopped'.");
              result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
              trajectory_server_.setAborted(result);
              return;
          }
      }


      last_nonstall_time_ = current_time;
      last_nonstall_angles_ = current_joint_angles;


      double previous_cmd[num_jaco_joints];
      for(int i = 0; i<num_jaco_joints; ++i)
        previous_cmd[i] = joint_pos[i];

      BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point, goal->trajectory.points){
    ROS_INFO("Trajectory Point");
    double joint_cmd[num_jaco_joints];
    for(int trajectory_index = 0; trajectory_index<goal->trajectory.joint_names.size(); ++trajectory_index){
      std::string joint_name = goal->trajectory.joint_names[trajectory_index];
      int joint_index = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), joint_name));
      if(joint_index >=0 && joint_index < num_jaco_joints){
        ROS_INFO("%s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, point.positions[trajectory_index]);
        joint_cmd[joint_index] = nearest_equivelent(point.positions[trajectory_index], previous_cmd[joint_index]);
      }
    }
        for(int i = 0; i<num_jaco_joints; ++i)
          previous_cmd[i] = joint_pos[i];


    AngularInfo angles;
    FingersPosition fingers;
    angles.Actuator1 = joint_cmd[0]*RAD_TO_DEG;
    angles.Actuator2 = joint_cmd[1]*RAD_TO_DEG;
    angles.Actuator3 = joint_cmd[2]*RAD_TO_DEG;
    angles.Actuator4 = joint_cmd[3]*RAD_TO_DEG;
    angles.Actuator5 = joint_cmd[4]*RAD_TO_DEG;
    angles.Actuator6 = joint_cmd[5]*RAD_TO_DEG;


    TrajectoryPoint jaco_point;
    memset(&jaco_point, 0, sizeof(jaco_point));

    jaco_point.LimitationsActive = false;
    jaco_point.Position.Delay = 0.0;
    jaco_point.Position.Type = ANGULAR_POSITION;
    jaco_point.Position.Actuators = angles;
    jaco_point.Position.HandMode = HAND_NOMOVEMENT;

    SendBasicTrajectory(jaco_point);
      }
    }

    ros::Rate rate(10);
    int trajectory_size;
    while(trajectory_size > 0){
      {
    boost::recursive_mutex::scoped_lock lock(api_mutex);

    TrajectoryFIFO Trajectory_Info;
    memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
    GetGlobalTrajectoryInfo(Trajectory_Info);
    trajectory_size = Trajectory_Info.TrajectoryCount;
      }
      ROS_INFO("%f, %f, %f, %f, %f, %f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
      rate.sleep();
    }
    ROS_INFO("Trajectory Control Complete.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    trajectory_server_.setSucceeded(result);
  }

  void JacoArmTrajectoryController::execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal){
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      EraseAllTrajectories();
      StopControlAPI();
      StartControlAPI();
      SetAngularControl();

      update_joint_states();

      AngularInfo angles;
      FingersPosition fingers;
      angles.Actuator1 = joint_pos[0];
      angles.Actuator2 = joint_pos[1];
      angles.Actuator3 = joint_pos[2];
      angles.Actuator4 = joint_pos[3];
      angles.Actuator5 = joint_pos[4];
      angles.Actuator6 = joint_pos[5];
      fingers.Finger1 = goal->command.position;
      fingers.Finger2 = goal->command.position;
      fingers.Finger3 = goal->command.position;

      TrajectoryPoint jaco_point;
      memset(&jaco_point, 0, sizeof(jaco_point));

      jaco_point.LimitationsActive = false;
      jaco_point.Position.Delay = 0.0;
      jaco_point.Position.Type = ANGULAR_POSITION;
      jaco_point.Position.Actuators = angles;
      jaco_point.Position.HandMode = POSITION_MODE;
      jaco_point.Position.Fingers = fingers;
      SendBasicTrajectory(jaco_point);
    }

    ros::Rate rate(10);
    int trajectory_size;
    while(trajectory_size > 0){
      {
    boost::recursive_mutex::scoped_lock lock(api_mutex);

    TrajectoryFIFO Trajectory_Info;
    memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
    GetGlobalTrajectoryInfo(Trajectory_Info);
    trajectory_size = Trajectory_Info.TrajectoryCount;
      }
      rate.sleep();
    }

    update_joint_states();
    control_msgs::GripperCommandResult result;
    result.position = joint_pos[6];
    result.position = joint_eff[6];
    result.stalled = false;
    result.reached_goal = true;
    gripper_server_.setSucceeded(result);
  }

}
