/*********************************************************************
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
 *********************************************************************/

// Original version: Melonee Wise <mwise@willowgarage.com>

#ifndef HEAD_SERVOING_CONTROLLER_H
#define HEAD_SERVOING_CONTROLLER_H

#include <ros/ros.h>
#include <pr2_controller_interface/controller.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>
#include <tf/transform_datatypes.h>
#include <robot_mechanism_controllers/joint_velocity_controller.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/scoped_ptr.hpp>
#include <math.h>


namespace controller
{

class HeadServoingController : public pr2_controller_interface::Controller
{
public:

  HeadServoingController();
  ~HeadServoingController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  void starting();
  void update();

  // input of the controller
  double pan_out_, tilt_out_;

private:

  pr2_mechanism_model::Robot* robot_;     
  ros::NodeHandle node_;
  std::string pan_link_name_, tilt_link_name_;
  pr2_mechanism_model::RobotState *robot_state_;
  
  double max_velocity_;
  double servo_gain_;
  ros::Subscriber sub_command_;
    

  void command(const sensor_msgs::JointStateConstPtr& command_msg);
  
  void pointHead(const tf::MessageNotifier<geometry_msgs::PointStamped>::MessagePtr& point_msg);
  void pointFrameOnHead(const tf::MessageNotifier<geometry_msgs::PointStamped>::MessagePtr& point_msg);

  

  tf::TransformListener tf_;
  boost::scoped_ptr<tf::MessageNotifier<geometry_msgs::PointStamped> > point_head_notifier_;
  boost::scoped_ptr<tf::MessageNotifier<geometry_msgs::PointStamped> > point_frame_on_head_notifier_;  

  // position controller
  JointVelocityController head_pan_controller_;
  JointVelocityController head_tilt_controller_;

};
}
#endif
