/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Peter Pastor */

// system includes

// ros includes
#include <pluginlib/class_list_macros.h>

// local includes
#include <experimental_controllers/joint_velocity_filtered_controller.h>

PLUGINLIB_REGISTER_CLASS(JointVelocityFilteredController, controller::JointVelocityFilteredController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

JointVelocityFilteredController::JointVelocityFilteredController()
: joint_state_(NULL), command_(0), robot_(NULL), last_time_(0), loop_count_(0)
{
}

JointVelocityFilteredController::~JointVelocityFilteredController()
{
  sub_command_.shutdown();
}

bool JointVelocityFilteredController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->getTime();

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("JointVelocityFilteredController could not find joint named \"%s\"\n",
              joint_name.c_str());
    return false;
  }

  pid_controller_ = pid;

  return true;
}

bool JointVelocityFilteredController::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    ROS_ERROR("JointVelocityFilteredController was not given a joint\n");
    return false;
  }

  const char *jn = j->Attribute("name");
  std::string joint_name = jn ? jn : "";

  TiXmlElement *p = j->FirstChildElement("pid");
  control_toolbox::Pid pid;
  if (p)
  {
    pid.initXml(p);
  }
  else
    ROS_ERROR("JointVelocityFilteredController's config did not specify the default pid parameters.\n");

  return init(robot, joint_name, pid);
}

bool JointVelocityFilteredController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;
  robot_ = robot;

  std::string joint_name;
  if (!node_.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(joint_state_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!pid_controller_.init(ros::NodeHandle(node_, "pid")))
    return false;

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "state", 1));

  sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &JointVelocityFilteredController::setCommandCB, this);

  velocity_.resize(1);
  if (!((filters::MultiChannelFilterBase<double>&)filter_velocity_).configure(1, n.getNamespace() + std::string("/filter_velocity"), n))
	{
		ROS_ERROR("Could not create velocity filter.");
  	return false;
	}

  return true;
}


void JointVelocityFilteredController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);

}

void JointVelocityFilteredController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string JointVelocityFilteredController::getJointName()
{
  return joint_state_->joint_->name;
}

// Set the joint velocity command
void JointVelocityFilteredController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current velocity command
void JointVelocityFilteredController::getCommand(double  & cmd)
{
  cmd = command_;
}

void JointVelocityFilteredController::update()
{
  assert(robot_ != NULL);
  ros::Time time = robot_->getTime();

  // filter the joint state velocity
  velocity_[0] = joint_state_->velocity_;
  filter_velocity_.update(velocity_, velocity_);

	filtered_velocity_ = velocity_[0];

  double error = velocity_[0] - command_;
  dt_ = time - last_time_;
  joint_state_->commanded_effort_ += pid_controller_.updatePid(error, dt_);

  if(loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = velocity_[0];
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = dt_.toSec();

      double dummy;
      getGains(controller_state_publisher_->msg_.p,
               controller_state_publisher_->msg_.i,
               controller_state_publisher_->msg_.d,
               controller_state_publisher_->msg_.i_clamp,
               dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;

  last_time_ = time;
}

void JointVelocityFilteredController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

} // namespace
