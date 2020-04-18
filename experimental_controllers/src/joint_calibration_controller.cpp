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

#include <experimental_controllers/joint_calibration_controller.h>
#include <ros/time.h>

using namespace std;
using namespace controller;


JointCalibrationController::JointCalibrationController()
  : state_(INITIALIZED), actuator_(NULL), joint_(NULL), transmission_(NULL)
{
}

JointCalibrationController::~JointCalibrationController()
{
}

bool JointCalibrationController::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  assert(config);

  TiXmlElement *cal = config->FirstChildElement("calibrate");
  if (!cal)
  {
    std::cerr<<"JointCalibrationController was not given calibration parameters"<<std::endl;
    return false;
  }

  if(cal->QueryDoubleAttribute("velocity", &search_velocity_) != TIXML_SUCCESS)
  {
    std::cerr<<"Velocity value was not specified\n";
    return false;
  }

  const char *joint_name = cal->Attribute("joint");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "Error: JointCalibrationController could not find joint \"%s\"\n",
            joint_name);
    return false;
  }

  const char *actuator_name = cal->Attribute("actuator");
  actuator_ = actuator_name ? robot->model_->getActuator(actuator_name) : NULL;
  if (!actuator_)
  {
    fprintf(stderr, "Error: JointCalibrationController could not find actuator \"%s\"\n",
            actuator_name);
    return false;
  }

  const char *transmission_name = cal->Attribute("transmission");
  transmission_ = transmission_name ? robot->model_->getTransmission(transmission_name) : NULL;
  if (!transmission_)
  {
    fprintf(stderr, "Error: JointCalibrationController could not find transmission \"%s\"\n",
            transmission_name);
    return false;
  }

  control_toolbox::Pid pid;
  TiXmlElement *pid_el = config->FirstChildElement("pid");
  if (!pid_el)
  {
    fprintf(stderr, "Error: JointCalibrationController was not given a pid element.\n");
    return false;
  }
  if (!pid.initXml(pid_el))
    return false;

  if (!vc_.init(robot, joint_name, pid))
    return false;

  return true;
}

void JointCalibrationController::update()
{
  assert(joint_);
  assert(actuator_);

  switch(state_)
  {
  case INITIALIZED:
    vc_.setCommand(0.0);
    state_ = BEGINNING;
    break;
  case BEGINNING:
    original_switch_state_ = actuator_->state_.calibration_reading_;
    vc_.setCommand(original_switch_state_ ? -search_velocity_ : search_velocity_);
    state_ = MOVING;
    break;
  case MOVING: {
    bool switch_state_ = actuator_->state_.calibration_reading_;
    if (switch_state_ != original_switch_state_)
    {
      Actuator a;
      pr2_mechanism_model::JointState j;
      std::vector<Actuator*> fake_a;
      std::vector<pr2_mechanism_model::JointState*> fake_j;
      fake_a.push_back(&a);
      fake_j.push_back(&j);

      // Where was the joint when the optical switch triggered?
      if (switch_state_ == true)
        fake_a[0]->state_.position_ = actuator_->state_.last_calibration_rising_edge_;
      else
        fake_a[0]->state_.position_ = actuator_->state_.last_calibration_falling_edge_;
      transmission_->propagatePosition(fake_a, fake_j);

      // What is the actuator position at the joint's zero?
      fake_j[0]->position_ = fake_j[0]->position_ - joint_->joint_->reference_position_;
      transmission_->propagatePositionBackwards(fake_j, fake_a);

      actuator_->state_.zero_offset_ = fake_a[0]->state_.position_;
      joint_->calibrated_ = true;

      state_ = CALIBRATED;
      vc_.setCommand(0.0);
    }
    break;
  }
  case CALIBRATED:
    break;
  }

  if (state_ != CALIBRATED)
    vc_.update();
}



JointCalibrationControllerNode::JointCalibrationControllerNode()
: robot_(NULL), last_publish_time_(0), pub_calibrated_(NULL)
{
}

JointCalibrationControllerNode::~JointCalibrationControllerNode()
{
  if (pub_calibrated_)
  {
    std::string topic = pub_calibrated_->topic_;
    delete pub_calibrated_;

    // I think we're all tired of having the "cal" topics cluttering
    // up rostopic and rosgraphviz.
    ros::Node::instance()->unadvertise(topic);
  }
}

void JointCalibrationControllerNode::update()
{
  c_.update();

  if (c_.calibrated())
  {
    if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime())
    {
      assert(pub_calibrated_);
      if (pub_calibrated_->trylock())
      {
        last_publish_time_ = robot_->getTime();
        pub_calibrated_->unlockAndPublish();
      }
    }
  }
}

bool JointCalibrationControllerNode::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  std::string topic = config->Attribute("name") ? config->Attribute("name") : "";
  if (topic == "")
  {
    fprintf(stderr, "No name given to JointCalibrationController\n");
    return false;
  }
  if (!c_.initXml(robot, config))
    return false;

  pub_calibrated_ = new realtime_tools::RealtimePublisher<std_msgs::Empty>(topic + "/calibrated", 1);

  return true;
}
