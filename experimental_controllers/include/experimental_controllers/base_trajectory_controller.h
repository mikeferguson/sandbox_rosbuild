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

#include <geometry_msgs/Twist.h>
#include <pr2_robot_actions/Pose2D.h>
#include <manipulation_msgs/JointTrajPoint.h>
#include <manipulation_msgs/JointTraj.h>
#include <trajectory/trajectory.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>

#include <angles/angles.h>

namespace pr2_mechanism_controllers
{
  class BaseTrajectoryController
  {
    public:

    BaseTrajectoryController(ros::NodeHandle &node, tf::TransformListener& tf);

    ~BaseTrajectoryController();

    trajectory::Trajectory::TPoint getPose2D(const tf::Stamped<tf::Pose> &pose);

    void pathCallback(const manipulation_msgs::JointTrajConstPtr &path_msg);

    void updateGlobalPose();

    double distance(double x1, double y1, double x2, double y2);

    bool goalPositionReached();

    bool goalOrientationReached();

    bool goalReached();

    void updatePath();

    void spin();

    void updateControl();

    geometry_msgs::Twist getCommand();

    private:
    tf::Stamped<tf::Pose> global_pose_;
    std::vector<control_toolbox::Pid> pid_;
    std::string global_frame_, robot_base_frame_;
    double controller_frequency_;
    std::string control_topic_name_, path_input_topic_name_;
    std::string trajectory_type_;
    manipulation_msgs::JointTraj path_msg_in_;
    manipulation_msgs::JointTraj path_msg_;

    ros::NodeHandle ros_node_;
//    ros::Node& ros_node_;
    tf::TransformListener& tf_;
    int dimension_;
    ros::Time current_time_;
    double sample_time_;
    ros::Time last_update_time_;
    ros::Time trajectory_start_time_;
    trajectory::Trajectory *trajectory_;
    bool stop_motion_;
    int stop_motion_count_;
    bool new_path_available_;

    std::vector<double> velocity_limits_;

    trajectory::Trajectory::TPoint goal_;
    trajectory::Trajectory::TPoint current_position_;
    double yaw_goal_tolerance_;
    double xy_goal_tolerance_;

    boost::mutex ros_lock_;
    ros::Time  path_updated_time_;
    double max_update_time_;

    geometry_msgs::Twist checkCmd(const geometry_msgs::Twist &cmd);

    double diagnostics_expected_publish_time_;

    ros::Time last_diagnostics_publish_time_;

    void publishDiagnostics(bool force);

    geometry_msgs::Twist cmd_vel_;
    double error_x_,error_y_,error_th_;

    std::string control_state_;

    ros::Subscriber trajectory_control_topic_;
    ros::Publisher base_control_topic_;
    ros::Publisher diagnostics_topic_;

 };
};

