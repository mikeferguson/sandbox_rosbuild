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

#include <experimental_controllers/joint_pd_controller.h>
#include <angles/angles.h>
#include "pluginlib/class_list_macros.h"

using namespace std;
using namespace controller;


JointPDController::JointPDController()
: joint_state_(NULL), robot_(NULL), last_time_(0), command_(0), command_dot_(0)
{
  pthread_mutex_init(&joint_pd_controller_lock_,NULL);
}

JointPDController::~JointPDController()
{
}

bool JointPDController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->getTime();

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    fprintf(stderr, "JointPDController could not find joint named \"%s\"\n",
            joint_name.c_str());
    return false;
  }

  pid_controller_ = pid;

  return true;

  command_= 0;
  command_dot_ = 0;
}

bool JointPDController::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
  assert(robot);

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointPDController was not given a joint\n");
    return false;
  }

  const char *jn = j->Attribute("name");
  std::string joint_name = jn ? jn : "";

  TiXmlElement *p = j->FirstChildElement("pid");
  control_toolbox::Pid pid;
  if (p)
    pid.initXml(p);
  else
    fprintf(stderr, "JointPDController's config did not specify the default pid parameters.\n");

  return init(robot, joint_name, pid);
}

void JointPDController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);
}

void JointPDController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string JointPDController::getJointName()
{
  return(joint_state_->joint_->name);
}

// Set the joint velocity command
void JointPDController::setCommand(double command, double command_dot)
{
  pthread_mutex_lock(&joint_pd_controller_lock_);
  command_t_ = command;
  command_dot_t_ = command_dot;
  pthread_mutex_unlock(&joint_pd_controller_lock_);
}

// Return the current  command
void JointPDController::getCommand(deprecated_msgs::JointCmd & cmd)
{
  pthread_mutex_lock(&joint_pd_controller_lock_);
  cmd.names[0]= joint_state_->joint_->name;
  cmd.positions[0] = command_t_;
  cmd.velocity[0] = command_dot_t_;
  pthread_mutex_unlock(&joint_pd_controller_lock_);
}

void JointPDController::reset()
{
  pid_controller_.reset();
}

void JointPDController::update()
{
  double error(0), error_dot(0);
  ros::Time time = robot_->getTime();

  if(pthread_mutex_trylock(&joint_pd_controller_lock_) == 0)
  {
    command_ = command_t_;
    command_dot_ = command_dot_t_;
    pthread_mutex_unlock(&joint_pd_controller_lock_);
  }

  error_dot = joint_state_->velocity_ - command_dot_;
//  error = joint_state_->position_ - command_;

  if(joint_state_->joint_->type == urdf::Joint::REVOLUTE)
  {
    angles::shortest_angular_distance_with_limits(command_, joint_state_->position_, joint_state_->joint_->limits->lower, joint_state_->joint_->limits->upper,error);

  }
  else if(joint_state_->joint_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(command_, joint_state_->position_);
  }
  else //prismatic
  {
    error = joint_state_->position_ - command_;
  }

//  error = joint_state_->position_ - command_;

  joint_state_->commanded_effort_ = pid_controller_.updatePid(error, error_dot, time - last_time_);
  last_time_ = time;
}


//------ Joint PD controller node --------

JointPDControllerNode::JointPDControllerNode()
{
  c_ = new JointPDController();
}

JointPDControllerNode::~JointPDControllerNode()
{
  delete c_;
}

void JointPDControllerNode::update()
{
  c_->update();
}

bool JointPDControllerNode::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
  service_prefix_ = config->Attribute("name");

  if (!c_->initXml(robot, config))
    return false;
  //subscriptions
  cmd_subscriber_ = node_.subscribe(service_prefix_ + "/set_command", 1, &JointPDControllerNode::setCommand, this);

  return true;
}

void JointPDControllerNode::setCommand(const deprecated_msgs::JointCmdConstPtr &msg)
{
  cmd_ = *msg;
  c_->setCommand(cmd_.positions[0],cmd_.velocity[0]);
}

