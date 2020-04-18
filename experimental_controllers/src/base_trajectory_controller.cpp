/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Sachin Chitta
*********************************************************************/
#include <experimental_controllers/base_trajectory_controller.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <ros/rate.h>

#include "ros/ros.h" 

namespace pr2_mechanism_controllers
{
  BaseTrajectoryController::BaseTrajectoryController(ros::NodeHandle &ros_node, tf::TransformListener& tf) : ros_node_(ros_node),tf_(tf)
  {

    ros::NodeHandle node_root;
    //get some parameters that will be global to the move base door node
    dimension_ = 3;

    control_topic_name_ = "cmd_vel";
    path_input_topic_name_ = "command";


    ros_node_.param("global_frame", global_frame_, std::string("odom_combined"));
    ros_node_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    ros_node_.param("controller_frequency", controller_frequency_, 100.0);
    ros_node_.param("diagnostics_expected_publish_time",diagnostics_expected_publish_time_,1.0);

    ros_node_.param("trajectory_type", trajectory_type_, std::string("linear"));

    double p_gain(0.0),i_gain(0.0),d_gain(0.0);
    ros_node_.param("x/p_gain", p_gain, 0.5);
    ros_node_.param("x/i_gain", i_gain, 0.1);
    ros_node_.param("x/d_gain", d_gain,0.0);
    control_toolbox::Pid pid_x(p_gain,i_gain,d_gain,0.5,-0.5);
    pid_.push_back(pid_x);

    ros_node_.param("y/p_gain", p_gain, 0.5);
    ros_node_.param("y/i_gain", i_gain, 0.1);
    ros_node_.param("y/d_gain", d_gain, 0.0);
    control_toolbox::Pid pid_y(p_gain,i_gain,d_gain,0.5,-0.5);
    pid_.push_back(pid_y);

    ros_node_.param("theta/p_gain", p_gain, 0.5);
    ros_node_.param("theta/i_gain", i_gain, 0.1);
    ros_node_.param("theta/d_gain", d_gain, 0.0);
    control_toolbox::Pid pid_t(p_gain,i_gain,d_gain,0.5,-0.5);
    pid_.push_back(pid_t);

    double vel;
    ros_node_.param("x/vel_limit", vel, 0.2);velocity_limits_.push_back(vel);
    ros_node_.param("y/vel_limit", vel, 0.2);velocity_limits_.push_back(vel);
    ros_node_.param("theta/vel_limit", vel, 0.2);velocity_limits_.push_back(vel);
    ros_node_.param("max_update_time", max_update_time_, 0.2);

    trajectory_control_topic_ = ros_node_.subscribe(path_input_topic_name_,1, &BaseTrajectoryController::pathCallback, this);//path_msg_in_
    base_control_topic_ = node_root.advertise<geometry_msgs::Twist>(control_topic_name_, 1);
    diagnostics_topic_ = node_root.advertise<diagnostic_msgs::DiagnosticArray> ("/diagnostics", 1) ;

    last_diagnostics_publish_time_ = ros::Time::now();
    current_time_ = ros::Time::now();
    last_update_time_ = ros::Time(0);
    trajectory_start_time_ = current_time_;
    stop_motion_ = true;
    stop_motion_count_ = 0;

    current_position_.setDimension(3);
    goal_.setDimension(3);
    updateGlobalPose();

    trajectory_ = new trajectory::Trajectory(dimension_);
    trajectory_->setMaxRates(velocity_limits_);
    trajectory_->setInterpolationMethod(trajectory_type_);
    trajectory_->setJointWraps(2);
    trajectory_->autocalc_timing_ = true;

    new_path_available_ = false;
  }

  BaseTrajectoryController::~BaseTrajectoryController()
  {
/*    ros_node_.unadvertise(control_topic_name_);
    ros_node_.unsubscribe(path_input_topic_name_);
    ros_node_.unadvertise("/diagnostics") ;
*/
  }

  trajectory::Trajectory::TPoint BaseTrajectoryController::getPose2D(const tf::Stamped<tf::Pose> &pose)
  {
    trajectory::Trajectory::TPoint tmp_pose;
    tmp_pose.setDimension(dimension_);
    double useless_pitch, useless_roll, yaw;
    pose.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
    tmp_pose.q_[0] = pose.getOrigin().x();
    tmp_pose.q_[1] = pose.getOrigin().y();
    tmp_pose.q_[2] = yaw;
    return tmp_pose;
  }

  void BaseTrajectoryController::pathCallback(const manipulation_msgs::JointTrajConstPtr &path_msg)
  {
    this->ros_lock_.lock();
    path_msg_ = *path_msg;
    this->ros_lock_.unlock();
    path_updated_time_ = ros::Time::now();
    new_path_available_ = true;
  }

  void BaseTrajectoryController::updateGlobalPose()
  {
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();

    try
    {
      tf_.transformPose(global_frame_, robot_pose, global_pose_);
    }
    catch(tf::LookupException& ex)
    {
      ROS_DEBUG("No Transform available Error: %s\n", ex.what());
    }
    catch(tf::ConnectivityException& ex)
    {
      ROS_DEBUG("Connectivity Error: %s\n", ex.what());
    }
    catch(tf::ExtrapolationException& ex)
    {
      ROS_DEBUG("Extrapolation Error: %s\n", ex.what());
    }
    current_position_ = getPose2D(global_pose_);
  }

  double BaseTrajectoryController::distance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  bool BaseTrajectoryController::goalPositionReached()
  {
    double dist = distance(current_position_.q_[0], current_position_.q_[1], goal_.q_[0], goal_.q_[1]);
    return fabs(dist) <= xy_goal_tolerance_;
  }

  bool BaseTrajectoryController::goalOrientationReached()
  {
    return fabs(angles::shortest_angular_distance(current_position_.q_[2], goal_.q_[2])) <= yaw_goal_tolerance_;
  }

  bool BaseTrajectoryController::goalReached()
  {
    return (goalPositionReached() && goalOrientationReached());
  }

  void BaseTrajectoryController::updatePath()
  {
    ros_lock_.lock();
    std::vector<trajectory::Trajectory::TPoint> waypoints;
    if((int)path_msg_.get_points_size() > 0)
    {
      if((int) path_msg_.points[0].get_positions_size() != dimension_)
      {
        stop_motion_ = true;
        control_state_ = "Input trajectory has wrong dimension - MOTION STOPPED";
        ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) path_msg_.points.size(), dimension_);
      }
      else
      {
        ROS_DEBUG("Dimension of input trajectory = %d",(int) path_msg_.get_points_size());
        int msg_size = (int)path_msg_.get_points_size();
        waypoints.resize(msg_size+1);
        waypoints[0].setDimension(dimension_);
        waypoints[0].q_[0] = current_position_.q_[0];
        waypoints[0].q_[1] = current_position_.q_[1];
        waypoints[0].q_[2] = current_position_.q_[2];
        waypoints[0].time_ = 0.0;
        for(int i=0; i < msg_size; i++)
        {
          waypoints[i+1].setDimension((int) dimension_);
          waypoints[i+1].q_[0] = path_msg_.points[i].positions[0];
          waypoints[i+1].q_[1] = path_msg_.points[i].positions[1];
          waypoints[i+1].q_[2] = path_msg_.points[i].positions[2];
          waypoints[i+1].time_ = 0.0;
        }
        goal_.q_[0] = path_msg_.points[msg_size-1].positions[0];
        goal_.q_[1] = path_msg_.points[msg_size-1].positions[1];
        goal_.q_[2] = path_msg_.points[msg_size-1].positions[2];
        stop_motion_ = false;
        trajectory_->setTrajectory(waypoints);
        trajectory_start_time_ = ros::Time::now();
      }
    }
    else
    {
      control_state_ = "No waypoints - MOTION STOPPED";
      ROS_DEBUG("Trajectory has no waypoints");
      stop_motion_ = true;
    }
    ros_lock_.unlock();
    return;
  }

  void BaseTrajectoryController::spin()
  {
    ros::Rate control_rate(controller_frequency_);
    while(ros_node_.ok())
    {
      ros::spinOnce();
      current_time_ = ros::Time::now();

      publishDiagnostics(false);

      updateGlobalPose();

      if(new_path_available_)
      {
        updatePath();
        new_path_available_ = false;
      }

      if(goalReached())//check for success
      {
        ROS_DEBUG("REACHED GOAL");
        control_state_ = "REACHED GOAL";
        stop_motion_ = true;
      }

      if ((current_time_-path_updated_time_).toSec() > max_update_time_)
      {
        ROS_DEBUG("No path update in %f seconds. Stopping motion.", (current_time_-path_updated_time_).toSec());
        stop_motion_ = true;
        control_state_ = "WATCHDOG - MOTION STOPPED";
      }

      ros::Time control_time = ros::Time::now();
      updateControl();
      ROS_DEBUG("Full control cycle: %.9f", (ros::Time::now() - control_time).toSec());

      last_update_time_ = current_time_;

      if (!control_rate.sleep())
        ROS_DEBUG("Control loop missed its desired cycle rate of %.4f Hz", controller_frequency_);
    }
  }

  void BaseTrajectoryController::publishDiagnostics(bool force)
  {
    if((ros::Time::now() - last_diagnostics_publish_time_).toSec() <= diagnostics_expected_publish_time_ && !force)
    {
      return;
    }

    diagnostic_msgs::DiagnosticArray message;
    std::vector<diagnostic_msgs::DiagnosticStatus> statuses;
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = ros_node_.getNamespace();
    status.summary(0, control_state_);

    status.add("Error.x", error_x_);
    status.add("Error.y", error_y_);
    status.add("Error.th", error_th_);
    status.add("Goal.x", goal_.q_[0]);
    status.add("Goal.y", goal_.q_[1]);
    status.add("Goal.th", goal_.q_[2]);
    status.add("Number of waypoints", path_msg_.get_points_size());
    status.add("Controller frequency (Hz)", controller_frequency_);
    status.add("Max update delta time (s)", max_update_time_);
    status.add("Control topic name", control_topic_name_);
    status.add("Global frame", global_frame_);
    status.add("Path input topic name", path_input_topic_name_);
    status.add("Trajectory type", trajectory_type_);

    statuses.push_back(status);

    message.header.stamp = ros::Time::now();
    message.status = statuses;
    diagnostics_topic_.publish(message);
    last_diagnostics_publish_time_ = message.header.stamp;
  }

  void BaseTrajectoryController::updateControl()
  {
    geometry_msgs::Twist cmd_vel;
    if(stop_motion_)
    {
      ROS_DEBUG("updateControl:: stopping motion");
      if(stop_motion_count_ < 3)
      {
        stop_motion_count_++;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        base_control_topic_.publish(cmd_vel);
        ROS_DEBUG("updateControl:: stop motion count: %d",stop_motion_count_);
      }
    }
    else
    {
      control_state_ = "ACTIVE - OK";
      stop_motion_count_ = 0;
      cmd_vel = getCommand();
      base_control_topic_.publish(cmd_vel);
      ROS_DEBUG("updateControl:: Publishing command: %f, %f, %f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
    }
  }

  geometry_msgs::Twist BaseTrajectoryController::getCommand()
  {
    double cmd[3];
    geometry_msgs::Twist cmd_vel;
    trajectory::Trajectory::TPoint desired_position;
    desired_position.setDimension(dimension_);
    double sample_time = (current_time_ - trajectory_start_time_).toSec();
    trajectory_->sample(desired_position,sample_time);
    double total_time = trajectory_->getTotalTime();
    double theta = current_position_.q_[2];
    double error_x = current_position_.q_[0] - desired_position.q_[0];
    double error_y = current_position_.q_[1] - desired_position.q_[1];
    double error_theta = angles::shortest_angular_distance(desired_position.q_[2],current_position_.q_[2]);
    ROS_DEBUG("Total time: %f, Sample: %f, Errors: %f, %f, %f, Feedforward: %f, %f, %f",total_time,sample_time,error_x,error_y,error_theta,desired_position.qdot_[0],desired_position.qdot_[1],desired_position.qdot_[2]);
    cmd[0] = pid_[0].updatePid(error_x, current_time_ - last_update_time_) + desired_position.qdot_[0];
    cmd[1] = pid_[1].updatePid(error_y, current_time_ - last_update_time_) + desired_position.qdot_[1];
    cmd[2] = pid_[2].updatePid(error_theta, current_time_ - last_update_time_) + desired_position.qdot_[2];

    //Transform the cmd back into the base frame
    cmd_vel.linear.x = cmd[0]*cos(theta) + cmd[1]*sin(theta);
    cmd_vel.linear.y = -cmd[0]*sin(theta) + cmd[1]*cos(theta);
    cmd_vel.angular.z = cmd[2];

    cmd_vel = checkCmd(cmd_vel);

    error_x_ = error_x;
    error_y_ = error_y;
    error_th_ = error_theta;

    cmd_vel_.linear.x = cmd_vel.linear.x;
    cmd_vel_.linear.y = cmd_vel.linear.y;
    cmd_vel_.angular.z = cmd_vel.angular.z;

    return cmd_vel;
  }

  geometry_msgs::Twist BaseTrajectoryController::checkCmd(const geometry_msgs::Twist &cmd)
  {
    geometry_msgs::Twist return_cmd = cmd;
    if(return_cmd.linear.x > velocity_limits_[0])
      return_cmd.linear.x = velocity_limits_[0];
    else if(return_cmd.linear.x < -velocity_limits_[0])
      return_cmd.linear.x = -velocity_limits_[0];

    if(return_cmd.linear.y > velocity_limits_[1])
      return_cmd.linear.y = velocity_limits_[1];
    else if(return_cmd.linear.y < -velocity_limits_[1])
      return_cmd.linear.y = -velocity_limits_[1];

    if(return_cmd.angular.z > velocity_limits_[2])
      return_cmd.angular.z = velocity_limits_[2];
    else if(return_cmd.angular.z < -velocity_limits_[2])
      return_cmd.angular.z = -velocity_limits_[2];

    return return_cmd;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_trajectory_controller");
  ros::NodeHandle nh("~");
  ros::spinOnce();
  tf::TransformListener tf(nh,ros::Duration(10),true);
  pr2_mechanism_controllers::BaseTrajectoryController base_trajectory_controller(nh, tf);
  base_trajectory_controller.spin();
  return(0);
}
