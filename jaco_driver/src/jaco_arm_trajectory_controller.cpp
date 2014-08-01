#include <jaco_driver/jaco_arm_trajectory_controller.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <boost/foreach.hpp>
#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"


namespace jaco{

  JacoArmTrajectoryController::JacoArmTrajectoryController(JacoComm &arm_comm, ros::NodeHandle nh) :
    arm_comm_(arm_comm),
    node_handle_(nh, "joint_angles"),
    trajectory_server_(nh, "arm_controller", boost::bind(&JacoArmTrajectoryController::execute_trajectory, this, _1), false),
    gripper_server_(nh, "fingers_controller", boost::bind(&JacoArmTrajectoryController::execute_gripper, this, _1), false),
    smooth_joint_trajectory_server(nh, "/setFollowTrajectory", boost::bind(&JacoArmTrajectoryController::execute_joint_trajectory, this, _1), false)
   {

    node_handle_.param<int>("num_jaco_finger_joints", num_jaco_finger_joints_, NUM_JACO_FINGER_JOINTS);
    node_handle_.param<int>("num_jaco_joints", num_jaco_joints_, NUM_JACO_JOINTS);
    node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
    node_handle_.param<double>("stall_threshold", stall_threshold_, 1.0);
    node_handle_.param<double>("rate_hz", rate_hz_, 10.0);

    double tolerance;
    node_handle_.param<double>("tolerance", tolerance, 2.0);
    tolerance_ = (float)tolerance;
    num_joints_ = num_jaco_joints_ + num_jaco_finger_joints_;

    for(int joint_id = 0; joint_id < num_jaco_joints_; ++joint_id){
      std::stringstream joint_name_stream;
      joint_name_stream << "mico_joint_" << (joint_id+1);
      std::string joint_name = joint_name_stream.str();
      joint_names.push_back(joint_name);
    }

    for(int finger_id = 0; finger_id < num_jaco_finger_joints_; ++finger_id){
      std::stringstream finger_name_stream;
      finger_name_stream << "mico_joint_finger_" << (finger_id+1);
      std::string finger_name = finger_name_stream.str();
      joint_names.push_back(finger_name);
    }
    
    joint_pos.resize(num_joints_);
    joint_vel.resize(num_joints_);
    joint_eff.resize(num_joints_);
    
    trajectory_server_.start();
    gripper_server_.start();
    smooth_joint_trajectory_server.start();
  }

  JacoArmTrajectoryController::~JacoArmTrajectoryController()
  {
  }
  
 
  static inline double simplify_angle(double angle)
  {
    double previous_rev = floor(angle/(2.0*M_PI))*2.0*M_PI;
    double next_rev = ceil(angle/(2.0*M_PI))*2.0*M_PI;
    double current_rev;
    if (fabs(angle - previous_rev) < fabs(angle - next_rev))
      return angle - previous_rev;
    return angle - next_rev;
  }

  float NormalizeAngle(float value)
  {
     while (value > 2*M_PI)
        value -= 2*M_PI;
     while (value < 0.0)
        value += 2*M_PI;

     return value;
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

  std::vector<double> VectorDifference(std::vector<double> &v1, std::vector<double> &v2)
  {
      std::vector<double> difference_vector;

      for(int i=0; i<v1.size(); i++)
      {
          difference_vector.at(i)=v1.at(i)-v2.at(i);
      }
      return difference_vector;
  }
  
  std::vector<double> JacoAnglesToVector(JacoAngles & joint_angles)
  {
      jaco_msgs::JointAngles jaco_angles = joint_angles.constructAnglesMsg();

      std::vector<double> angle_vector(6);
      angle_vector[0] = jaco_angles.joint1;
      angle_vector[1] = jaco_angles.joint2;
      angle_vector[2] = jaco_angles.joint3;
      angle_vector[3] = jaco_angles.joint4;
      angle_vector[4] = jaco_angles.joint5;
      angle_vector[5] = jaco_angles.joint6;
      return angle_vector;
  }


  void JacoArmTrajectoryController::execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
      control_msgs::FollowJointTrajectoryFeedback feedback;
      control_msgs::FollowJointTrajectoryResult result;
      control_msgs::FollowJointTrajectoryActionFeedback action_feedback;

      JacoAngles current_joint_angles;
      ros::Time current_time = ros::Time::now();

      std::vector<JacoAngles> jacoTrajectory;
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
      catch(const std::exception& e)
      {
          ROS_ERROR_STREAM(e.what());
      }

      last_nonstall_time_ = current_time;
      last_nonstall_angles_ = current_joint_angles;


      double previous_cmd[goal->trajectory.joint_names.size()];
      for(int i = 0; i<goal->trajectory.joint_names.size(); ++i)
          previous_cmd[i] = joint_pos[i];

      BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point, goal->trajectory.points)
      {
          ROS_INFO("Trajectory Point");
          double joint_cmd[goal->trajectory.joint_names.size()];


          //Transform the joint trajectory point a jaco_angle point by iterating over the indices of the entries in the
          //joint trajectory point and looking up the joint name in the internal joint_names list to find the index

          for(int trajectory_index = 0; int joint_index = std::distance(joint_names.begin(),joint_names.end()); ++trajectory_index)
          {
              std::string joint_name = goal->trajectory.joint_names[trajectory_index]; // The name of the current joint in the trajectory point
              joint_index = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), joint_name)); // Get the index of that joint name in the internal list

              control_msgs::FollowJointTrajectoryFeedback feedback;
              control_msgs::FollowJointTrajectoryResult result;
              JacoAngles current_joint_angles;
              ros::Time current_time = ros::Time::now();

              std::vector<JacoAngles> jacoTrajectory;
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
              catch(const std::exception& e)
              {
                  ROS_ERROR_STREAM(e.what());
              }

              last_nonstall_time_ = current_time;
              last_nonstall_angles_ = current_joint_angles;


              double previous_cmd[num_jaco_joints_];
              for(int i = 0; i<num_jaco_joints_; ++i)
                  previous_cmd[i] = joint_pos[i];

              BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point, goal->trajectory.points)
              {
                  ROS_INFO("Trajectory Point");
                  double joint_cmd[num_jaco_joints_];


                  //Transform the joint trajectory point a jaco_angle point by iterating over the indices of the entries in the
                  //joint trajectory point and looking up the joint name in the internal joint_names list to find the index

                  for(int trajectory_index = 0; trajectory_index<goal->trajectory.joint_names.size(); ++trajectory_index)
                  {
                      std::string joint_name = goal->trajectory.joint_names[trajectory_index]; // The name of the current joint in the trajectory point
                      joint_index = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), joint_name)); // Get the index of that joint name in the internal list

                      //If the joint name was found, pack the joint position from the trajectory point into a temporary vector in expected order for the jaco
                      if(joint_index >=0 && joint_index < num_jaco_joints_){
                          ROS_INFO("%s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, point.positions[trajectory_index]);
                          joint_cmd[joint_index] = nearest_equivelent(simplify_angle(point.positions[trajectory_index]), previous_cmd[joint_index]);
                      }
                  }//for

                  JacoAngles jAngles;
                  jAngles.Actuator1 = joint_cmd[0];
                  jAngles.Actuator2 = joint_cmd[1];
                  jAngles.Actuator3 = joint_cmd[2];
                  jAngles.Actuator4 = joint_cmd[3];
                  jAngles.Actuator5 = joint_cmd[4];
                  jAngles.Actuator6 = joint_cmd[5];

                  JacoAngles target(jAngles);
                  jacoTrajectory.push_back(target);
              }

              //Send the trajectory to the jaco arm
              arm_comm_.setJointAngles(jacoTrajectory.at(trajectory_index));
              JacoAngles final_goal = jacoTrajectory.back();

              //Monitor the arm to hand feedback and results.current_joint_angles[current_joint_angles.size()-1]
              while (true)
              {
                  ros::spinOnce();
                  //trajectory server ?+action server
                  if ((trajectory_server_.isPreemptRequested() || action_feedback.status.status == actionlib_msgs::GoalStatus::PREEMPTING) || !ros::ok())
                  {
                      arm_comm_.stopAPI();
                      arm_comm_.startAPI();
                      trajectory_server_.setPreempted(result);
                  }
                  else if (arm_comm_.isStopped())
                  {
                      trajectory_server_.setAborted(result);
                      return;
                  }

                  arm_comm_.getJointAngles(current_joint_angles);
                  current_time = ros::Time::now();

                  control_msgs::FollowJointTrajectoryFeedback feedback;

                  feedback.joint_names = joint_names;
                  std::vector<double> final_goal_vec = JacoAnglesToVector(final_goal);
                  std::vector<double> current_angle_vec = JacoAnglesToVector(current_joint_angles);
                  std::vector<double> error = VectorDifference(final_goal_vec, current_angle_vec);

                  feedback.desired.positions = final_goal_vec;

                  feedback.actual.positions = current_angle_vec;
                  feedback.error.positions = error;
                  trajectory_server_.publishFeedback(feedback);

                  //Check if the action has succeeded
                  if(final_goal.isCloseToOther(current_joint_angles, tolerance_))
                  {
                      trajectory_server_.setSucceeded(result);
                      return;
                  }

                  else if (!last_nonstall_angles_.isCloseToOther(current_joint_angles, stall_threshold_))
                  {
                      // Check if we are outside of a potential stall condition
                      last_nonstall_time_ = current_time;
                      last_nonstall_angles_ = current_joint_angles;
                  }

                  //Check if we took too long
                  else if ((current_time - last_nonstall_time_).toSec() > stall_interval_seconds_)
                  {
                      arm_comm_.stopAPI();
                      trajectory_server_.setPreempted(result);
                      result.error_code = result.GOAL_TOLERANCE_VIOLATED;
                      return;
                  }
              }

              if(joint_index >=0 && joint_index < num_jaco_joints_){
                  ROS_INFO("%s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, point.positions[trajectory_index]);
                  joint_cmd[joint_index] = nearest_equivelent(simplify_angle(point.positions[trajectory_index]), previous_cmd[joint_index]);
              }
          }//for

          JacoAngles jAngles;
          jAngles.Actuator1 = joint_cmd[0];
          jAngles.Actuator2 = joint_cmd[1];
          jAngles.Actuator3 = joint_cmd[2];
          jAngles.Actuator4 = joint_cmd[3];
          jAngles.Actuator5 = joint_cmd[4];
          jAngles.Actuator6 = joint_cmd[5];

          JacoAngles target(jAngles);
          jacoTrajectory.push_back(target);
      }

      //Send the trajectory to the jaco arm
      JacoAngles final_goal = jacoTrajectory.back();
      arm_comm_.setJointAngles(final_goal);  

      //Monitor the arm to send feedback and results.current_joint_angles[current_joint_angles.size()-1]
      while (true)
      {
          ros::spinOnce();
          //trajectory server ?+action server
          if ((action_feedback.status.status == actionlib_msgs::GoalStatus::PREEMPTING) || !ros::ok())
          {
              arm_comm_.stopAPI();
              arm_comm_.startAPI();
              trajectory_server_.setPreempted(result);
          }
          else if (arm_comm_.isStopped())
          {
              trajectory_server_.setAborted(result);
              return;
          }

          arm_comm_.getJointAngles(current_joint_angles);
          current_time = ros::Time::now();
          feedback.joint_names = joint_names;
          std::vector<double> final_goal_vec = JacoAnglesToVector(final_goal);
          std::vector<double> current_angle_vec = JacoAnglesToVector(current_joint_angles);
          std::vector<double> error = VectorDifference(final_goal_vec, current_angle_vec);

          feedback.desired.positions = final_goal_vec;
          feedback.actual.positions = current_angle_vec;
          feedback.error.positions = error;
          trajectory_server_.publishFeedback(feedback);

          //Check if the action has succeeded
          if(final_goal.isCloseToOther(current_joint_angles, tolerance_))
          {
              trajectory_server_.setSucceeded(result);
              return;
          }

          else if (!last_nonstall_angles_.isCloseToOther(current_joint_angles, stall_threshold_))
          {
              // Check if we are outside of a potential stall condition
              last_nonstall_time_ = current_time;
              last_nonstall_angles_ = current_joint_angles;
          }

          //Check if we took too long
          else if ((current_time - last_nonstall_time_).toSec() > stall_interval_seconds_)
          {
              arm_comm_.stopAPI();
              arm_comm_.startAPI();
              trajectory_server_.setPreempted(result);
              result.error_code = result.GOAL_TOLERANCE_VIOLATED;
              return;
          }
      }

  }

  jaco_msgs::JointAngles toJacoMsgJointAngles(sensor_msgs::JointState & state, std::vector<std::string> & joint_names)
  {


      int joint_indices[NUM_JACO_JOINTS];
      //extract joint indices
      for (unsigned int ji = 0; ji < NUM_JACO_JOINTS; ++ji)
          joint_indices[ji] = std::distance(state.name.begin(), std::find(state.name.begin(), state.name.end(), joint_names[ji])); // Get the index of that joint name in the internal list
      jaco_msgs::JointAngles position_data_msg;
      position_data_msg.joint1 = state.position[joint_indices[0]];
      position_data_msg.joint2 = state.position[joint_indices[1]];
      position_data_msg.joint3 = state.position[joint_indices[2]];
      position_data_msg.joint4 = state.position[joint_indices[3]];
      position_data_msg.joint5 = state.position[joint_indices[4]];
      position_data_msg.joint6 = state.position[joint_indices[5]];
      return position_data_msg;
  }

  sensor_msgs::JointState toJointState(jaco_msgs::JointAngles & position_data_msg, std::vector<std::string> &  joint_names, const std::vector<std::string> * required_joint_order)
  {
    sensor_msgs::JointState state;
    int joint_indices[NUM_JACO_JOINTS];
    for (unsigned int ji = 0; ji < NUM_JACO_JOINTS; ++ji)
    {
        if (required_joint_order)
            joint_indices[ji] = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), (*required_joint_order)[ji])); // Get the index of that joint name in the internal list
        else
            joint_indices[ji] = ji;
    }

    state.name = joint_names;
    state.position.resize(6,0);
    state.position[joint_indices[0]] = position_data_msg.joint1;
    state.position[joint_indices[1]] = position_data_msg.joint2;
    state.position[joint_indices[2]] = position_data_msg.joint3;
    state.position[joint_indices[3]] = position_data_msg.joint4;
    state.position[joint_indices[4]] = position_data_msg.joint5;
    state.position[joint_indices[5]] = position_data_msg.joint6;
    return state;
  }
  sensor_msgs::JointState toNormalizedJointState(jaco_msgs::JointAngles & position_data_msg, std::vector<std::string> &  joint_names, const std::vector<std::string> * required_joint_order)
  {
    sensor_msgs::JointState state;
    int joint_indices[NUM_JACO_JOINTS];
    for (unsigned int ji = 0; ji < NUM_JACO_JOINTS; ++ji)
    {
        if (required_joint_order)
            joint_indices[ji] = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), (*required_joint_order)[ji])); // Get the index of that joint name in the internal list
        else
            joint_indices[ji] = ji;
    }

    state.name = joint_names;
    state.position.resize(6,0);
    state.position[joint_indices[0]] = NormalizeAngle(position_data_msg.joint1);
    state.position[joint_indices[1]] = position_data_msg.joint2;
    state.position[joint_indices[2]] = position_data_msg.joint3;
    state.position[joint_indices[3]] = NormalizeAngle(position_data_msg.joint4);
    state.position[joint_indices[4]] = NormalizeAngle(position_data_msg.joint5);
    state.position[joint_indices[5]] = NormalizeAngle(position_data_msg.joint6);
    return state;
  }


  sensor_msgs::JointState getError (sensor_msgs::JointState desired_joint_state, sensor_msgs::JointState current_joint_state)
  {
      sensor_msgs::JointState error;
      error.position.resize(6,0);
      for (unsigned int i=0; i<NUM_JACO_JOINTS; i++)
      {
          error.position[i] = nearest_equivelent(simplify_angle(desired_joint_state.position[i]), current_joint_state.position[i]) - current_joint_state.position[i];
      }
      return error;

  }

  bool isCloseEnough(sensor_msgs::JointState desired_joint_state, sensor_msgs::JointState current_joint_state, double threshold)
  {
      sensor_msgs::JointState error=getError(desired_joint_state, current_joint_state);
      double total_error=0;
      for(unsigned int i=0; i<NUM_JACO_JOINTS; i++)
      {
          total_error+=fabs(error.position[i]);
      }
      return total_error<threshold;
  }


  sensor_msgs::JointState toJointState(JacoAngles & position_data, std::vector<std::string> & joint_names, const std::vector<std::string> * required_joint_order)
  {
      jaco_msgs::JointAngles angleMsg = position_data.constructAnglesMsg();
      return toJointState(angleMsg, joint_names, required_joint_order);

  }
  sensor_msgs::JointState toNormalizedJointState(JacoAngles & position_data, std::vector<std::string> & joint_names, const std::vector<std::string> * required_joint_order)
  {
      jaco_msgs::JointAngles angleMsg = position_data.constructAnglesMsg();
      return toNormalizedJointState(angleMsg, joint_names, required_joint_order);

  }


  JacoAngles toJacoAngles(sensor_msgs::JointState & state, std::vector<std::string> & joint_names)
  {
      jaco_msgs::JointAngles position_data_msg = toJacoMsgJointAngles(state, joint_names);
      return JacoAngles(position_data_msg);
  }

  JacoAngles toJacoVelocities(sensor_msgs::JointState & state, std::vector<std::string> & joint_names)
    {

      JacoAngles offsetAngles;

      offsetAngles.Actuator1 = -state.position[0];
      offsetAngles.Actuator2 = state.position[1];
      offsetAngles.Actuator3 = -state.position[2];
      offsetAngles.Actuator4 = -state.position[3];
      offsetAngles.Actuator5 = -state.position[4];
      offsetAngles.Actuator6 = -state.position[5];
      return offsetAngles;
    }
  /*RosPosition*/
  //make error.position

  void JacoArmTrajectoryController::execute_joint_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
      sensor_msgs::JointState current_joint_state;
      sensor_msgs::JointState desired_joint_state;
      JacoAngles position_data;
      desired_joint_state.name = goal->trajectory.joint_names;
      desired_joint_state.position.resize(6,0);

      desired_joint_state.position = goal->trajectory.points[0].positions ;
      arm_comm_.getJointAngles(position_data);
      current_joint_state = toJointState(position_data,joint_names, &goal->trajectory.joint_names);
      for(unsigned int i=0; i<NUM_JACO_JOINTS; i++)
      {
          desired_joint_state.position[i]=nearest_equivelent(desired_joint_state.position[i], current_joint_state.position[i]);
      }
      if(!isCloseEnough(desired_joint_state, current_joint_state, .03))
      {
          control_msgs::FollowJointTrajectoryResult result;
          result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
          smooth_joint_trajectory_server.setAborted(result);
          if(!arm_comm_.isStopped())
            arm_comm_.stopAPI();
          ROS_ERROR_STREAM("Initial arm state not valid.");
          return;
      }


      float trajectoryPoints[NUM_JACO_JOINTS][goal->trajectory.points.size()];
      int numPoints = goal->trajectory.points.size();
      //get trajectory data
      for (unsigned int i = 0; i < numPoints; i ++)
      {
        for (unsigned int j = 0; j < NUM_JACO_JOINTS; j ++)
        {
          trajectoryPoints[j][i] = goal->trajectory.points.at(i).positions.at(j);
        }
      }

      //initialize arrays needed to fit a smooth trajectory to the given points
      ecl::Array<double> timePoints(numPoints);
      timePoints[0] = 0.0;
      std::vector< ecl::Array<double> > jointPoints;
      jointPoints.resize(NUM_JACO_JOINTS);
      float prevPoint[NUM_JACO_JOINTS];

      //Assign initial point in trajectory
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
      {
        jointPoints[i].resize(numPoints);
        jointPoints[i][0] = trajectoryPoints[i][0];
        prevPoint[i] = trajectoryPoints[i][0];
      }


      //determine time component of trajectories for each joint
      for (unsigned int i = 1; i < numPoints; i ++)
      {
        float maxTime = 0.0;

        //First iterate through all of the joint positions and find the one with the largest
        //change relative to it's allowed maximum velocity. The first three joints have a lower
        //maximum velocity than the last three. Scale the time taken by the motion so that the
        //velocity limits of all the joints are respected.
        //
        for (unsigned int j = 0; j < NUM_JACO_JOINTS; j ++)
        {
          //calculate approximate time required to move to the next position
          float time = fabs(trajectoryPoints[j][i] - prevPoint[j]);
          if (j <= 2)
            time /= LARGE_ACTUATOR_VELOCITY;
          else
            time /= SMALL_ACTUATOR_VELOCITY;

          if (time > maxTime)
            maxTime = time;

          jointPoints[j][i] = trajectoryPoints[j][i];
          prevPoint[j] = trajectoryPoints[j][i];
        }

        timePoints[i] = timePoints[i-1] + maxTime*TIME_SCALING_FACTOR;
      }

      double max_curvature = 6.0;  //NOTE: this may need adjustment depending on the source of the trajectory points; this value seems to work fine for trajectories generated by MoveIt!.
      std::vector<ecl::SmoothLinearSpline> splines;
      splines.resize(6);
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
      {
        ecl::SmoothLinearSpline tempSpline(timePoints, jointPoints[i], max_curvature);
        splines.at(i) = tempSpline;
      }
    
    //control loop
    bool trajectoryComplete = false;
    double startTime = ros::Time::now().toSec();
    double t = 0;
    sensor_msgs::JointState error;
    error.name = goal->trajectory.joint_names;
    error.position.resize(NUM_JACO_JOINTS,0);

    sensor_msgs::JointState prevError;
    prevError.name = goal->trajectory.joint_names;
    prevError.position.resize(NUM_JACO_JOINTS,0);
    double current_joint_pos[NUM_JACO_JOINTS];
    AngularInfo angular_velocities;
    TrajectoryPoint trajPoint;
    trajPoint.InitStruct();
    trajPoint.Position.Type = ANGULAR_VELOCITY;
    trajPoint.Position.HandMode = HAND_NOMOVEMENT;

    sensor_msgs::JointState command;
    command.position.resize(6,0);
    command.name = error.name;   
    while (!trajectoryComplete)
    {
      //check for preempt requests from clients
      if (smooth_joint_trajectory_server.isPreemptRequested() || !ros::ok())
      {

        //preempt action server
        smooth_joint_trajectory_server.setPreempted();
        ROS_INFO("Joint trajectory server preempted by client");
        
        return;
      }
      if(arm_comm_.isStopped())
          arm_comm_.startAPI();

      arm_comm_.getJointAngles(position_data);
      current_joint_state = toJointState(position_data,joint_names, &goal->trajectory.joint_names);

    
      //get time for trajectory
      t = std::min(ros::Time::now().toSec() - startTime, timePoints.at(timePoints.size() - 1));

      for (int i = 0; i < NUM_JACO_JOINTS; ++i)
          desired_joint_state.position[i] = nearest_equivelent(splines.at(i)(t), current_joint_state.position[i]);
      if (t == timePoints.at(timePoints.size() - 1) && isCloseEnough(desired_joint_state, current_joint_state, .03))
        {

          arm_comm_.stopAPI();
          trajectoryComplete = true;
          ROS_INFO("Trajectory complete!");
          break;
        }
      error = getError(desired_joint_state, current_joint_state);
      //Otherwise, populate the current command
      for(int i=0; i< NUM_JACO_JOINTS; i++)
        {
            command.position[i] = KP*error.position[i] +KV*(error.position[i]-prevError.position[i])*RAD_TO_DEG;
        }

      angular_velocities = toJacoVelocities(command, joint_names);


        //send the velocity command
        if(fabs(angular_velocities.Actuator1)>50  || fabs(angular_velocities.Actuator2)>50 || fabs(angular_velocities.Actuator3)>50 || fabs(angular_velocities.Actuator4)>50 || fabs(angular_velocities.Actuator5)>50 || fabs(angular_velocities.Actuator6)>50)
        {
            ROS_ERROR("Angular velocity exceeds predefined velocity threshold.");
            ROS_ERROR_STREAM("1: " <<angular_velocities.Actuator1 << " 2: "
                                     << angular_velocities.Actuator2 << " 3: "
                                     << angular_velocities.Actuator3 << " 4: "
                                     << angular_velocities.Actuator4 <<" 5: "
                                     << angular_velocities.Actuator5 <<" 6: "
                                     << angular_velocities.Actuator6);
           // control_msgs::FollowJointTrajectoryResult result;
           // result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
          //  arm_comm_.stopAPI();
         //   smooth_joint_trajectory_server.setAborted(result);
          //  return;
        }
        else{
            ROS_INFO_STREAM("1: " <<angular_velocities.Actuator1 << " 2: "
                                     << angular_velocities.Actuator2 << " 3: "
                                     << angular_velocities.Actuator3 << " 4: "
                                     << angular_velocities.Actuator4 <<" 5: "
                                     << angular_velocities.Actuator5 <<" 6: "
                                     << angular_velocities.Actuator6);
        }
        //if(!arm_comm_.isStopped())
        //    arm_comm_.stopAPI();
        arm_comm_.setJointVelocities(angular_velocities);
       prevError = error;

    }
    
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smooth_joint_trajectory_server.setSucceeded(result);
  }


  void JacoArmTrajectoryController::execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal){} /*
      {
          //took out recursive mutex
          EraseAllTrajectories();
          StopControlAPI();
          StartControlAPI();
          SetAngularControl();

          //took out update_joint_states()

          AngularInfo angles;
          FingersPosition fingers;
          angles.Actuator= joint_pos[0];
          angles.Actuator2 = joint_pos[1];
          angles.Actuator3 = joint_pos[2];
          angles.Actuator4 = joint_pos[3];
          angles.Actuator5 = joint_pos[4];
          angles.Actuator6 = joint_pos[5];
          fingers.Finger1 = goal->command.position;
          fingers.Finger2 = goal->command.position;
          fingers.Finger3 = goal->command.position;  //Do we want finger 3??

          TrajectoryPoint jaco_point;
          memset(&jaco_point, 0, sizeof(jaco_point));

          jaco_point.LimitationsActive =false;
          jaco_point.Position.Delay = 0.0;
          jaco_point.Position.Type = ANGULAR_POSITION;
          jaco_point.Position.Actuators = angles;
          jaco_point.Position.HandMode = POSITION_MODE;
          jaco_point.Position.Fingers = fingers;
          SendBasicTrajectory(jaco_point);
      }

      ros::Rate rate(10);  //this is ROS
      int trajetory_size;
      bool execution_succeeded=true;


      if(execution_succeeded)
      {
          update_joint_states();
          control_msgs::GripperCommand result;
          result.position = joint_pos[6];
          result.position = joint_eff[6];
          result.stalled = false;
          result.reached_goal =true;
          gripper_server_.setSucceeded(result);
      }


  }

  /*
  void JacoArmTrajectoryController::execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal){
      {
          boost::recursive_mutex::scoped_lock lock(api_mutex);
          EraseAllTrajectories();
          StopControlAPI();
          StartControlAPI();4:223.89
          SetAngularControl();

          //update_joint_states();

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

      //update_joint_states();
      control_msgs::GripperCommandResult result;
      result.position = joint_pos[6];
      result.position = joint_eff[6];
      result.stalled = false;
      result.reached_goal = true;
      gripper_server_.setSucceeded(result);
  }*/
}
