/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 */

#include "experimental_controllers/probe.h"
#include "experimental_controllers/JointTuningMsg.h"

namespace controller {


Probe::Probe()
  : robot_(NULL), joint_(NULL)
{
}

Probe::~Probe()
{
  pub_probe_.shutdown();
}

bool Probe::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
  robot_ = robot;
  node_ = ros::NodeHandle(config->Attribute("name"));

  const char* joint_name = config->Attribute("joint");
  if (!joint_name) {
    ROS_ERROR("No joint name given");
    return false;
  }

  joint_ = robot->getJointState(joint_name);
  if (!joint_) {
    ROS_ERROR("Could not find joint \"%s\"", joint_name);
    return false;
  }

  pub_probe_ = node_.advertise<experimental_controllers::JointTuningMsg>("/probe/" + joint_->joint_->name_, 100);

  ROS_WARN("Probe initialized on joint %s.  Prepare for realtime breakage.", joint_name);

  return true;
}

void Probe::update()
{
  experimental_controllers::JointTuningMsg msg;
  msg.position = joint_->position_;
  msg.velocity = joint_->velocity_;
  msg.torque = joint_->commanded_effort_;
  msg.torque_measured = joint_->applied_effort_;
  msg.time_step = robot_->getTime().toSec();
  pub_probe_.publish(msg);
}

}
