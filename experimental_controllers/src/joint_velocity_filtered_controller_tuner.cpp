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
#include <math.h>

// ros includes
#include <pluginlib/class_list_macros.h>

// local includes
#include <experimental_controllers/joint_velocity_filtered_controller_tuner.h>

PLUGINLIB_REGISTER_CLASS(JointVelocityFilteredControllerTuner, controller::JointVelocityFilteredControllerTuner, pr2_controller_interface::Controller)

using namespace std;

namespace controller
{

JointVelocityFilteredControllerTuner::JointVelocityFilteredControllerTuner() :
	initialized_(false), joint_velocity_filtered_controller_(NULL), robot_state_(NULL), sine_wave_value_(0)
{

}

JointVelocityFilteredControllerTuner::~JointVelocityFilteredControllerTuner()
{

}

bool JointVelocityFilteredControllerTuner::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node_handle)
{

	assert(robot_state);
	robot_state_ = robot_state;
	node_handle_ = node_handle;

	initialized_ = false;

	std::string controller_name;
	if (!node_handle_.getParam("controller_to_tune", controller_name))
	{
		ROS_ERROR("No controller given (namespace: %s)", node_handle_.getNamespace().c_str());
		return false;
	}

	ros::NodeHandle controller_handle(node_handle_, controller_name);
	joint_velocity_filtered_controller_ = new JointVelocityFilteredController();
	ros::NodeHandle joint_controller_handle(ros::NodeHandle("/"), controller_name);
	if (!joint_velocity_filtered_controller_->init(robot_state_, joint_controller_handle))
	{
		ROS_ERROR("Could not initialize controller named %s.", controller_name.c_str());
		return false;
	}

	if (!node_handle_.getParam("sine_frequency", sine_frequency_))
	{
		ROS_ERROR("No sine_frequency given (namespace: %s)", node_handle_.getNamespace().c_str());
		return false;
	}
	if (!node_handle_.getParam("sine_amplitude", sine_amplitude_))
	{
		ROS_ERROR("No sine_amplitude given (namespace: %s)", node_handle_.getNamespace().c_str());
		return false;
	}
	if (!node_handle_.getParam("sine_offset", sine_offset_))
	{
		ROS_ERROR("No sine_offset given (namespace: %s)", node_handle_.getNamespace().c_str());
		return false;
	}

	initialized_ = true;
	return true;
}

bool JointVelocityFilteredControllerTuner::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
	ros::NodeHandle n(config->Attribute("name"));
	return init(robot, n);
}

void JointVelocityFilteredControllerTuner::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
	if (initialized_)
	{
		joint_velocity_filtered_controller_->setGains(p, i, d, i_max, i_min);
	}
}

void JointVelocityFilteredControllerTuner::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
	if (initialized_)
	{
		joint_velocity_filtered_controller_->getGains(p, i, d, i_max, i_min);
	}
}

// Set the joint velocity command
void JointVelocityFilteredControllerTuner::setCommand(double cmd)
{
	if (initialized_)
	{
		joint_velocity_filtered_controller_->command_ = cmd;
	}
}

// Return the current velocity command
void JointVelocityFilteredControllerTuner::getCommand(double &cmd)
{
	if (initialized_)
	{
		cmd = joint_velocity_filtered_controller_->command_;
	}
}

void JointVelocityFilteredControllerTuner::starting()
{
	actual_time_ = 0;
	if (initialized_)
	{
          joint_velocity_filtered_controller_->starting();
	}
}

void JointVelocityFilteredControllerTuner::update()
{

	time_ = robot_state_->getTime();
	actual_time_ += (time_.toSec() - last_time_.toSec());

	sine_wave_value_ = sine_offset_ + (sine_amplitude_ * sin(sine_frequency_ * (2.0 * M_PI) * actual_time_));

	joint_velocity_filtered_controller_->setCommand(sine_wave_value_);
	joint_velocity_filtered_controller_->update();

	last_time_ = time_;
}

std::string JointVelocityFilteredControllerTuner::getJointName()
{
	if (initialized_)
	{
		return joint_velocity_filtered_controller_->getJointName();
	}
	return std::string("controller not initialized");
}

} // namespace
