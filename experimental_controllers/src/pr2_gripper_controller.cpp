/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 /*
 * Author: Sachin Chitta and Matthew Piccoli
 */

#include <experimental_controllers/pr2_gripper_controller.h>
#include <fstream> //TODO:now that I have something better, should I delete this?
#include <cmath>
#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(Pr2GripperControllerOld, controller::Pr2GripperControllerOld, pr2_controller_interface::Controller)

using namespace controller;

Pr2GripperControllerOld::Pr2GripperControllerOld()
{
}

Pr2GripperControllerOld::~Pr2GripperControllerOld()
{
  node_.shutdown();
  service_thread_.join();
}

bool Pr2GripperControllerOld::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node)
{
  double p, i, d, i1, i2;
  pthread_mutex_init(&pr2_gripper_controller_lock_,NULL);
  node_ = node;
  grasp_cmd_.cmd = "move";
  grasp_cmd_.val = 0.0;
  new_cmd_available_ = false;
  robot_state_ = robot_state;
  //name_ = config->Attribute("name"); //"l_gripper" or "r_gripper" expected
  name_ = node.getNamespace();
//  pr2_mechanism_model::Link *link = robot_state_->model_->getLink(name_ + "_link");
  std::string joint_name, fingertip_sensor_topic;
  node_.param<std::string>("joint_name",joint_name,"r_gripper_joint");
  node_.param<std::string>("fingertip_sensor_topic",fingertip_sensor_topic,"/pressure/r_gripper_motor");
  joint_ = NULL;
  joint_ = robot_state_->getJointState(joint_name);
  if(!joint_)
  {
    ROS_ERROR("Could not initialize joint");
    return false;
  }
  if(!joint_controller_.init(robot_state_,joint_name))
  {
    ROS_ERROR("Could not initialize joint controller");
    return false;
  }
  if(!joint_->joint_->limits){
    ROS_ERROR("No joint limits specified");
    return false;
  }

  node_.param<double>("publish_rate",publish_rate_,25.0);
  node_.param<double>("default_effort",default_effort_,joint_->joint_->limits->effort);
  node_.param<double>("timeout", timeout_, 0.0);
  node_.param<double>("break_stiction_amplitude", break_stiction_amplitude_, 0.0);
  node_.param<double>("break_stiction_period", break_stiction_period_, 0.1);
  node_.param<double>("break_stiction_velocity", break_stiction_velocity_, 0.005);
  node_.param<double>("proportional_offset", proportional_offset_, 0.01);
  node_.param<double>("stopped_threshold", stopped_threshold_, 0.0001);
  node_.param<double>("timeout_duration_", timeout_duration_, 10.0);
  node_.param<double>("timeout_duration_steady", timeout_duration_steady_, 3.0);
  node_.param<double>("force", force_, 20.0);
  node_.param<double>("force_increase", force_increase_, 5.0);
  node_.param<double>("vel_p", p, 15000.0);
  node_.param<double>("vel_i", i, 25.0);
  node_.param<double>("vel_d", d, 0.0);
  node_.param<double>("vel_iclamp_high", i1, 100.0);
  node_.param<double>("vel_iclamp_low", i2, -100.0);
  node_.param<double>("default_low_speed", default_low_speed_, 0.01);
  node_.param<double>("default_high_speed", default_high_speed_, 0.015);
  node_.param<int>("contact_threshold", contact_threshold_, 1000);
  node_.param<int>("contact_threshold_individual", contact_threshold_individual_, 100);
  node_.param<int>("num_pressure_pads_front", num_pressure_pads_front_, 15);
  node_.param<int>("num_pressure_pads_side", num_pressure_pads_side_, 7);
  node_.param<std::string>("break_stiction_type", break_stiction_type_, "none");
  node_.param<std::string>("fingertip_sensor_topic", fingertip_sensor_topic_,"/pressure/r_gripper_motor");

  ros::AdvertiseServiceOptions ops = ros::AdvertiseServiceOptions::create<experimental_controllers::GraspClosedLoop>("/grasp_closed_loop", boost::bind(&Pr2GripperControllerOld::grasp_cl_srv, this, _1, _2), ros::VoidPtr(), &service_queue_);
  grasp_service_ = node_.advertiseService(ops);
  service_thread_ = boost::thread(boost::bind(&Pr2GripperControllerOld::callbackThread, this));
  cmd_sub_ = node_.subscribe<experimental_controllers::GripperControllerCmd>("cmd", 1, &Pr2GripperControllerOld::command_callback, this);
  pressure_sub_ = node_.subscribe<pr2_msgs::PressureState>(fingertip_sensor_topic, 1, &Pr2GripperControllerOld::pressure_state_callback, this);
  state_publisher_.reset(new realtime_tools::RealtimePublisher <experimental_controllers::GripperControllerState> (name_+"/state", 1));

  pressure_state_.l_finger_tip.resize(num_pressure_pads_front_+num_pressure_pads_side_);
  pressure_state_.r_finger_tip.resize(num_pressure_pads_front_+num_pressure_pads_side_);
  fingertip_sensor_start0_.resize(num_pressure_pads_front_);
  fingertip_sensor_start1_.resize(num_pressure_pads_front_);
  fingertip_sensor_first_peak0_.resize(num_pressure_pads_front_);
  fingertip_sensor_first_peak1_.resize(num_pressure_pads_front_);
  fingertip_sensor_first_steady0_.resize(num_pressure_pads_front_);
  fingertip_sensor_first_steady1_.resize(num_pressure_pads_front_);
  fingertip_sensor_second_peak0_.resize(num_pressure_pads_front_);
  fingertip_sensor_second_peak1_.resize(num_pressure_pads_front_);
  fingertip_sensor_second_steady0_.resize(num_pressure_pads_front_);
  fingertip_sensor_second_steady1_.resize(num_pressure_pads_front_);
  fingertip_sensor_sides_start0_.resize(num_pressure_pads_side_);
  fingertip_sensor_sides_start1_.resize(num_pressure_pads_side_);
  p_i_d_.initPid(p,i,d,i1,i2);

  return true;
}

bool Pr2GripperControllerOld::initXml(pr2_mechanism_model::RobotState *robot_state, TiXmlElement *config)
{
  ros::NodeHandle n(config->Attribute("name"));
  return init(robot_state, n);
}

void Pr2GripperControllerOld::update()
{
  //do nothing if the joint is not calibrated
  if (!joint_->calibrated_)
  {
    ROS_INFO("gripper not calibrated!");
    return;  // motor's not calibrated
  }

  ros::Time current_time = robot_state_->getTime();
  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&pr2_gripper_controller_lock_) == 0) //the callback is not writing to grasp_cmd_
    {
      if(grasp_cmd_desired_.cmd.compare("Event")) //If the incoming message is an event, don't copy the command
      {
        grasp_cmd_desired_.cmd = grasp_cmd_.cmd;
        grasp_cmd_desired_.start = grasp_cmd_.start;
        grasp_cmd_desired_.end = grasp_cmd_.end;
        grasp_cmd_desired_.time = grasp_cmd_.time;
        grasp_cmd_desired_.val = grasp_cmd_.val;
        closed_loop_grasp_state = unstarted;
        //ROS_INFO("Copying command, val is %f", grasp_cmd_desired_.val);
      }
      new_cmd_available_ = false;
      pthread_mutex_unlock(&pr2_gripper_controller_lock_);
      if(grasp_cmd_desired_.cmd.compare("step") == 0) //in here because it depends on former joint_controller_.command_, can't be changing with every update
      {
        velocity_mode_ = false;
        joint_controller_.command_ = effortLimit(stepMove(grasp_cmd_desired_.val));
        last_commanded_command = joint_controller_.command_;
      }
    }
  }

  //check for timeout
  if((current_time - cmd_received_timestamp_).toSec() <= timeout_ || timeout_ == 0.0) //continue with what you were doing
  {
    last_commanded_command = parseMessage(grasp_cmd_desired_);
    if(!velocity_mode_)
    {
      p_i_d_.reset();
      double direction = 1.0;
      last_commanded_command = effortLimit(last_commanded_command); //set value
      if(last_commanded_command < 0.0)
      {
        direction = -1.0;
      }
      if(break_stiction_type_.compare("sine") == 0 && fabs(joint_->velocity_) < break_stiction_velocity_ && last_commanded_command != 0.0)
      {
        joint_controller_.command_ = last_commanded_command + direction*(sin(2.0*M_PI*(current_time - cmd_received_timestamp_).toSec()/break_stiction_period_)*break_stiction_amplitude_ + break_stiction_amplitude_);
      }
      else if(break_stiction_type_.compare("ramp") == 0 && fabs(joint_->velocity_) < break_stiction_velocity_ && last_commanded_command != 0.0)
      {
        joint_controller_.command_ = last_commanded_command + direction*rampMove(0.0, break_stiction_amplitude_, break_stiction_period_, 0.0);
      }
      else
      {
        joint_controller_.command_ = last_commanded_command;
      }
      last_velocity_time_ = ros::Time(0.0);
    }
    else
    {
      if(last_velocity_time_ == ros::Time(0.0)) //it was just in effort mode
      {
        if(last_commanded_command >= 0.0)
        {
          p_i_d_.setCurrentCmd(force_);
        }
        else
        {
          p_i_d_.setCurrentCmd(-1.0*force_);
        }
        joint_controller_.command_ = p_i_d_.updatePid(joint_->velocity_ - last_commanded_command, ros::Duration(0.0));
      }
      else
        joint_controller_.command_ = p_i_d_.updatePid(joint_->velocity_ - last_commanded_command, current_time - last_time_);
      last_velocity_time_ = current_time;
    }

    joint_controller_.update(); //update value
  }
  else //if timed out, don't do anything
  {
    //stop motor
    //set value
    joint_controller_.command_ = 0.0;
    last_commanded_command = 0.0;
    //update value
    joint_controller_.update();
  }

  //Publish state
  if(current_time > last_published_time_ + ros::Duration(1.0/publish_rate_) && state_publisher_->trylock())
  {
    state_publisher_->msg_.joint_commanded_effort = joint_->commanded_effort_;
    state_publisher_->msg_.joint_measured_effort = joint_->measured_effort_;
    state_publisher_->msg_.joint_name = joint_->joint_->name;
    state_publisher_->msg_.joint_velocity = joint_->velocity_;
    state_publisher_->msg_.joint_position = joint_->position_;
    state_publisher_->unlockAndPublish() ;
    last_published_time_ = current_time;
  }
  last_time_ = current_time;
}

void Pr2GripperControllerOld::starting()
{
  last_time_ = robot_state_->getTime();
  cmd_received_timestamp_ = robot_state_->getTime();
  return true;
}

void Pr2GripperControllerOld::stopping()
{
  /*  if(state_publisher_)
  {
    state_publisher_->stop();
    delete state_publisher_;
    }*/
}

double Pr2GripperControllerOld::rampMove(double start_force, double end_force, double time, double hold)
{
  double del_force = end_force - start_force;
  double del_time = (robot_state_->getTime() - cmd_received_timestamp_).toSec();
  if(del_time > time)
  	return hold;
  return start_force + del_time/time*del_force;
}

double Pr2GripperControllerOld::stepMove(double step_size)
{
  return last_commanded_command + step_size;
}

double Pr2GripperControllerOld::grasp(bool service_callback, bool closed_loop)
{
  //starting grasp
  if(closed_loop_grasp_state == unstarted)
  {
    //reset variables
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      fingertip_sensor_start0_[i]=0;
      fingertip_sensor_start1_[i]=0;
      fingertip_sensor_first_peak0_[i]=0;
      fingertip_sensor_first_peak1_[i]=0;
      fingertip_sensor_first_steady0_[i]=0;
      fingertip_sensor_first_steady1_[i]=0;
      fingertip_sensor_second_peak0_[i]=0;
      fingertip_sensor_second_peak1_[i]=0;
      fingertip_sensor_second_steady0_[i]=0;
      fingertip_sensor_second_steady1_[i]=0;
    }
    //remember values from edges of fingertips
    for(int i = 0; i < num_pressure_pads_side_; i++)
    {
      fingertip_sensor_sides_start0_[i] = 0;
      fingertip_sensor_sides_start1_[i] = 0;
    }
    position_first_contact=-1;
    //position_second_contact=-1;
    position_first_compression=-1;
    //position_second_compression=-1;
    spring_const=-1;
    peak_force_first_grasp = 0;
    //peak_force_second_grasp = 0;
    peak_distance_first_grasp = 0.0;
    //peak_distance_second_grasp = 0.0;
    grasp_open_close_timestamp = robot_state_->getTime();
    closed_loop_grasp_state = open0;
    velocity_mode_ = false;
    if(service_callback && closed_loop_trail_ > 0)
      return (force_+force_increase_*(closed_loop_trail_-1));
    return default_effort_;
  }

  //open
  else if(closed_loop_grasp_state == open0)
  {
    //check for any timeouts
    if(grasp_open_close_timestamp + ros::Duration(timeout_duration_) < robot_state_->getTime())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      if(service_callback)
      {
        service_response_.result = 1;
        service_response_.distance[closed_loop_trail_] = joint_->position_;
        service_response_.effort[closed_loop_trail_] = default_effort_;
        service_response_.velocity = default_low_speed_;
        service_response_.time[closed_loop_trail_] = -1.0;
        //don't do time_to_first because it's closed_loop_trial_ - 1
        service_response_.force_peak0[closed_loop_trail_] = -1.0;
        service_response_.force_steady0[closed_loop_trail_] = -1.0;
        service_response_.force_peak1[closed_loop_trail_] = -1.0;
        service_response_.force_steady1[closed_loop_trail_] = -1.0;
        service_response_.distance_compressed[closed_loop_trail_] = -1.0;
        service_response_.distance_peak[closed_loop_trail_] = -1.0;
        service_response_.stiffness[closed_loop_trail_] = -1.0;
      }
      velocity_mode_ = false;
      return default_effort_;
    }

    if (service_callback_ && closed_loop_trail_ > 0 && joint_->position_ - .01 > service_response_.distance[0]) //if it's outside of the gripper
    {
      service_response_.time_to_return[closed_loop_trail_ - 1] = (robot_state_->getTime() - grasp_open_close_timestamp).toSec();
      //read the gripper pads to zero them
      for(int i = 0; i < num_pressure_pads_front_; i++) //the front pads are 7-21
      {
        fingertip_sensor_start0_[i] = pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
        fingertip_sensor_start1_[i] = pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      }
      for(int i = 0; i < num_pressure_pads_side_; i++)
      {
        fingertip_sensor_sides_start0_[i] = pressure_state_.l_finger_tip[i];
        fingertip_sensor_sides_start1_[i] = pressure_state_.r_finger_tip[i];
      }
      //chage state
      closed_loop_grasp_state = close0_closing;
      grasp_open_close_timestamp = robot_state_->getTime();
      velocity_mode_ = true;
      return -1.0*default_low_speed_;
    }
    //if the gripper is done opening
    else if(joint_->velocity_ < stopped_threshold_ && joint_->velocity_ > -1.0*stopped_threshold_ && grasp_open_close_timestamp + ros::Duration(.1) < robot_state_->getTime())
    {
      if(service_callback_ && closed_loop_trail_ > 0)
      {
        //if it stopped while it is within touching distance of the bottle + 1 mm
        if(joint_->position_ - .01 < service_response_.distance[0])
        {
          service_response_.time_to_return[closed_loop_trail_ - 1] = -1;
          return default_effort_;
        }
      }

      //read the gripper pads to zero them
      for(int i = 0; i < num_pressure_pads_front_; i++) //the front pads are 7-21
      {
        fingertip_sensor_start0_[i] = pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
        fingertip_sensor_start1_[i] = pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      }
      for(int i = 0; i < num_pressure_pads_side_; i++)
      {
        fingertip_sensor_sides_start0_[i] = pressure_state_.l_finger_tip[i];
        fingertip_sensor_sides_start1_[i] = pressure_state_.r_finger_tip[i];
      }
      //chage state
      closed_loop_grasp_state = close0_closing;
      grasp_open_close_timestamp = robot_state_->getTime();
      velocity_mode_ = true;
      return -1.0*default_low_speed_;
    }
    else
    {
      velocity_mode_ = false;
      if(service_callback && closed_loop_trail_ > 0)
        return (force_+force_increase_*(closed_loop_trail_-1));
      return default_effort_;
    }
  }

  //close with low force
  else if(closed_loop_grasp_state == close0_closing)
  {
    int starting_force_sum0 = 0;
    int current_force_sum0 = 0;
    int starting_force_sum1 = 0;
    int current_force_sum1 = 0;
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      current_force_sum0 += pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
      current_force_sum1 += pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      starting_force_sum0 += fingertip_sensor_start0_[i];
      starting_force_sum1 += fingertip_sensor_start1_[i];
    }
    //break if there is contact on edges of fingertips before first contact (cl only)
    if(closed_loop)
    {
      for(int i = 0; i < num_pressure_pads_side_; i++)
      {
        if(pressure_state_.l_finger_tip[i] - fingertip_sensor_sides_start0_[i] > contact_threshold_individual_ || pressure_state_.r_finger_tip[i] - fingertip_sensor_sides_start1_[i] > contact_threshold_individual_)
        {
          ROS_WARN("grasp failed due to impact on the fingertip sides");
          closed_loop_grasp_state = failed;
          //report that there was a failure
          if(service_callback)
          {
            service_response_.result = 2;
            service_response_.distance[closed_loop_trail_] = joint_->position_;
            service_response_.effort[closed_loop_trail_] = default_effort_;
            service_response_.velocity = default_low_speed_;
            for(int j = 0; j < num_pressure_pads_side_; j++)
            {
              service_response_.fingertip_profile0[j] = pressure_state_.l_finger_tip[j] - fingertip_sensor_sides_start0_[j];
              service_response_.fingertip_profile1[j] = pressure_state_.r_finger_tip[j] - fingertip_sensor_sides_start1_[j];
            }
            service_response_.time[closed_loop_trail_] = -1.0;
            //don't do time_to_first because it's closed_loop_trial_ - 1
            service_response_.force_peak0[closed_loop_trail_] = -1.0;
            service_response_.force_steady0[closed_loop_trail_] = -1.0;
            service_response_.force_peak1[closed_loop_trail_] = -1.0;
            service_response_.force_steady1[closed_loop_trail_] = -1.0;
            service_response_.distance_compressed[closed_loop_trail_] = -1.0;
            service_response_.distance_peak[closed_loop_trail_] = -1.0;
            service_response_.stiffness[closed_loop_trail_] = -1.0;
          }
          velocity_mode_ = false;
          return default_effort_;
        }
      }
    }
    //break if we've closed too much for the expected object (cl only)
    if(closed_loop && service_callback && joint_->position_ < service_request_.distance - service_request_.distance_tolerance)
    {
      ROS_WARN("grasp failed due to closing too far without impacting object");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      service_response_.result = 3;
      service_response_.distance[closed_loop_trail_] = joint_->position_;
      service_response_.effort[closed_loop_trail_] = default_effort_;
      service_response_.velocity = default_low_speed_;
      service_response_.time[closed_loop_trail_] = -1.0;
      //don't do time_to_first because it's closed_loop_trial_ - 1
      service_response_.force_peak0[closed_loop_trail_] = -1.0;
      service_response_.force_steady0[closed_loop_trail_] = -1.0;
      service_response_.force_peak1[closed_loop_trail_] = -1.0;
      service_response_.force_steady1[closed_loop_trail_] = -1.0;
      service_response_.distance_compressed[closed_loop_trail_] = -1.0;
      service_response_.distance_peak[closed_loop_trail_] = -1.0;
      service_response_.stiffness[closed_loop_trail_] = -1.0;
      velocity_mode_ = false;
      return default_effort_;
    }
    //check for any timeouts
    if(grasp_open_close_timestamp + ros::Duration(timeout_duration_) < robot_state_->getTime())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      if(service_callback)
      {
        service_response_.result = 1;
        service_response_.distance[closed_loop_trail_] = joint_->position_;
        service_response_.effort[closed_loop_trail_] = default_effort_;
        service_response_.velocity = default_low_speed_;
        service_response_.time[closed_loop_trail_] = -1.0;
        //don't do time_to_first because it's closed_loop_trial_ - 1
        service_response_.force_peak0[closed_loop_trail_] = -1.0;
        service_response_.force_steady0[closed_loop_trail_] = -1.0;
        service_response_.force_peak1[closed_loop_trail_] = -1.0;
        service_response_.force_steady1[closed_loop_trail_] = -1.0;
        service_response_.distance_compressed[closed_loop_trail_] = -1.0;
        service_response_.distance_peak[closed_loop_trail_] = -1.0;
        service_response_.stiffness[closed_loop_trail_] = -1.0;
      }
      velocity_mode_ = false;
      return default_effort_;
    }
    //record first contact info
    else if(current_force_sum0 > starting_force_sum0 + contact_threshold_ && current_force_sum1 > starting_force_sum1 + contact_threshold_)
    {
      position_first_contact = joint_->position_;
      closed_loop_grasp_state = close0_contact;
      grasp_open_close_timestamp = robot_state_->getTime();
      if(closed_loop && service_callback && joint_->position_ > service_request_.distance + service_request_.distance_tolerance)
      {
        ROS_WARN("grasp failed due to closing too little before impacting object");
        closed_loop_grasp_state = failed;
        //report that there was a failure
        service_response_.result = 4;
        service_response_.distance[closed_loop_trail_] = joint_->position_;
        service_response_.effort[closed_loop_trail_] = default_effort_;
        service_response_.velocity = default_low_speed_;
        service_response_.time[closed_loop_trail_] = -1.0;
        //don't do time_to_first because it's closed_loop_trial_ - 1
        service_response_.force_peak0[closed_loop_trail_] = -1.0;
        service_response_.force_steady0[closed_loop_trail_] = -1.0;
        service_response_.force_peak1[closed_loop_trail_] = -1.0;
        service_response_.force_steady1[closed_loop_trail_] = -1.0;
        service_response_.distance_compressed[closed_loop_trail_] = -1.0;
        service_response_.distance_peak[closed_loop_trail_] = -1.0;
        service_response_.stiffness[closed_loop_trail_] = -1.0;
        velocity_mode_ = false;
        return default_effort_;
      }
    }
    velocity_mode_ = true;
    return -1.0*default_low_speed_;
  }

  //continue closing
  else if(closed_loop_grasp_state == close0_contact)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      current_force_sum += pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
      current_force_sum += pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      starting_force_sum += fingertip_sensor_start0_[i];
      starting_force_sum += fingertip_sensor_start1_[i];
    }
    //record peak contact info
    if(peak_force_first_grasp < current_force_sum - starting_force_sum)
    {
      peak_distance_first_grasp = joint_->position_;
      peak_force_first_grasp = current_force_sum - starting_force_sum;
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        fingertip_sensor_first_peak0_[i] = pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
        fingertip_sensor_first_peak1_[i] = pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      }
    }

    //record time to first steady state info
    if(service_callback && closed_loop_trail_ > 0 && service_response_.time_to_first[closed_loop_trail_-1] < 0.0 && joint_->position_ < service_response_.distance_compressed[0])
    {
      service_response_.time_to_first[closed_loop_trail_ - 1] = (robot_state_->getTime()- grasp_open_close_timestamp).toSec();
      ROS_WARN("Assigning time to first %i to be %f", closed_loop_trail_-1, (robot_state_->getTime()- grasp_open_close_timestamp).toSec());
    }

    //record final contact info
    if(joint_->velocity_ < stopped_threshold_ && joint_->velocity_ > -1.0*stopped_threshold_)
    {
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        fingertip_sensor_first_steady0_[i] = pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
        fingertip_sensor_first_steady1_[i] = pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      }
      position_first_compression = joint_->position_;

      //if the grasp was of a known object, and want to compare for accuracy or unknown object, and want to store info
      if(service_callback)
      {
        service_response_.result = 0;
        service_response_.distance[closed_loop_trail_] = position_first_contact;
        if(closed_loop)
          service_response_.effort[closed_loop_trail_] = service_request_.effort;
        else
          service_response_.effort[closed_loop_trail_] = -1.0*(force_+force_increase_*closed_loop_trail_);
        service_response_.velocity = default_low_speed_;
        int force_peak0 = 0;
        int force_steady0 = 0;
        int force_peak1 = 0;
        int force_steady1 = 0;
        for(int i = 0; i < num_pressure_pads_front_; i++)
        {
          service_response_.fingertip_profile0[i+num_pressure_pads_side_] = fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          service_response_.fingertip_profile1[i+num_pressure_pads_side_] = fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
          force_peak0 += fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i];
          force_steady0 += fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          force_peak1 += fingertip_sensor_first_peak1_[i] - fingertip_sensor_start1_[i];
          force_steady1 += fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
        }
        for(int i = 0; i < num_pressure_pads_side_; i++)
        {
          service_response_.fingertip_profile0[i] = pressure_state_.l_finger_tip[i]-fingertip_sensor_sides_start0_[i];
          service_response_.fingertip_profile1[i] = pressure_state_.r_finger_tip[i]-fingertip_sensor_sides_start1_[i];
        }
        service_response_.time[closed_loop_trail_] = (robot_state_->getTime()- grasp_open_close_timestamp).toSec();
        service_response_.force_peak0[closed_loop_trail_] = force_peak0;
        service_response_.force_steady0[closed_loop_trail_] = force_steady0;
        service_response_.force_peak1[closed_loop_trail_] = force_peak1;
        service_response_.force_steady1[closed_loop_trail_] = force_steady1;
        service_response_.distance_compressed[closed_loop_trail_] = joint_->position_;
        service_response_.distance_peak[closed_loop_trail_] = peak_distance_first_grasp;
        service_response_.stiffness[closed_loop_trail_] = joint_->position_/position_first_contact;
        if(!closed_loop || service_response_.stiffness[closed_loop_trail_] < service_request_.stiffness +service_request_.stiffness_threshold && service_response_.stiffness[closed_loop_trail_] > service_request_.stiffness - service_request_.stiffness_threshold)
        {
          if(closed_loop_trail_ < service_request_.trials - 1 )
          {
            closed_loop_trail_++;
            closed_loop_grasp_state = unstarted;
            velocity_mode_ = false;
            return (force_+force_increase_*(closed_loop_trail_-1));
          }
          closed_loop_grasp_state = complete;
          service_success_ = true;
          velocity_mode_ = false;
          ROS_INFO("grasp succeeded");
          return service_request_.effort;
        }
        else
        {
          ROS_WARN("grasp failed due to the stiffness of the object being outside of tolerance");
          closed_loop_grasp_state = failed;
          //report that there was a failure
          service_response_.result = 5;
          velocity_mode_ = false;
          return default_effort_;
        }
      }
      else
      {
        //closed_loop_grasp_state = open1;
        //grasp_open_close_timestamp = robot_state_->getTime();
        closed_loop_grasp_state = complete;
      }
    }
    else if(grasp_open_close_timestamp + ros::Duration(timeout_duration_steady_) < robot_state_->getTime())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report that there was a failure
      if(service_callback)
      {
        service_response_.result = 1;
        service_response_.distance[closed_loop_trail_] = joint_->position_;
        service_response_.effort[closed_loop_trail_] = default_effort_;
        service_response_.velocity = default_low_speed_;
        service_response_.time[closed_loop_trail_] = -1.0;
        //don't do time_to_first because it's closed_loop_trial_ - 1
        service_response_.force_peak0[closed_loop_trail_] = -1.0;
        service_response_.force_steady0[closed_loop_trail_] = -1.0;
        service_response_.force_peak1[closed_loop_trail_] = -1.0;
        service_response_.force_steady1[closed_loop_trail_] = -1.0;
        service_response_.distance_compressed[closed_loop_trail_] = -1.0;
        service_response_.distance_peak[closed_loop_trail_] = -1.0;
        service_response_.stiffness[closed_loop_trail_] = -1.0;
      }
    }
    velocity_mode_ = false;
    return -1.0*(force_+force_increase_*closed_loop_trail_);
  }
/*
  //open
  else if(closed_loop_grasp_state == open1)
  {
    //if I'm done opening
    if(robot_state_->getTime() > grasp_open_close_timestamp.toSec() + break_stiction_period_)
    {
      closed_loop_grasp_state = close1_closing;
      grasp_open_close_timestamp = robot_state_->getTime();
      velocity_mode_ = true;
      return -1.0*default_high_speed_;
    }
    //else open
    else
    {
      velocity_mode_ = false;
      return default_effort_;
    }
  }

  //close with higher force
  //TODO::is this necessary?
  else if(closed_loop_grasp_state == close1_closing)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      current_force_sum += pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
      current_force_sum += pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      starting_force_sum += fingertip_sensor_start0_[i];
      starting_force_sum += fingertip_sensor_start1_[i];
    }
    //check for any timeouts
    if(grasp_open_close_timestamp + timeout_duration_ < robot_state_->getTime())
    {
      ROS_WARN("grasp failed due to a timeout");
      closed_loop_grasp_state = failed;
      //report faliure!
      if(closed_loop)
      {
        service_response_.distance = position_first_contact;
        service_response_.effort = default_effort_;
        int force_peak0 = 0;
        int force_steady0 = 0;
        int force_peak1 = 0;
        int force_steady1 = 0;
        for(int i = 0; i < num_pressure_pads_front_; i++)
        {
          service_response_.fingertip_profile0[i+num_pressure_pads_side_] = fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          service_response_.fingertip_profile1[i+num_pressure_pads_side_] = fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
          force_peak0 += fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i];
          force_steady0 += fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          force_peak1 += fingertip_sensor_first_peak1_[i] - fingertip_sensor_start1_[i];
          force_steady1 += fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
        }
        for(int i = 0; i < num_pressure_pads_side_; i++)
        {
          service_response_.fingertip_profile0[i] = pressure_state_.l_finger_tip[i]-fingertip_sensor_sides_start0_[i];
          service_response_.fingertip_profile1[i] = pressure_state_.r_finger_tip[i]-fingertip_sensor_sides_start1_[i];
        }
        service_response_.stiffness = 0.0;
        service_response_.force_peak0 = force_peak0;
        service_response_.force_steady0 = force_steady0;
        service_response_.force_peak1 = force_peak1;
        service_response_.force_steady1 = force_steady1;
        service_response_.distance_compressed_first = position_first_compression;
        service_response_.distance_compressed_second = -1.0;
      }
      velocity_mode_ = false;
      return default_effort_;
    }
    //record first contact info
    else if(current_force_sum > starting_force_sum + contact_threshold_)
    {
      position_second_contact = joint_->position_;
      closed_loop_grasp_state = close1_contact;
      grasp_open_close_timestamp = robot_state_->getTime();
    }
    velocity_mode_ = true;
    return -1.0*default_high_speed_;
  }

  //continue closing
  else if(closed_loop_grasp_state == close1_contact)
  {
    double starting_force_sum = 0;
    double current_force_sum = 0;
    for(int i = 0; i < num_pressure_pads_front_; i++)
    {
      current_force_sum += pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
      current_force_sum += pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      starting_force_sum += fingertip_sensor_start0_[i];
      starting_force_sum += fingertip_sensor_start1_[i];
    }
    //record peak contact info
    if(peak_force_second_grasp < current_force_sum - starting_force_sum)
    {
      peak_force_second_grasp = current_force_sum - starting_force_sum;
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        fingertip_sensor_second_peak0_[i] = pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
        fingertip_sensor_second_peak1_[i] = pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      }
    }
    //record final contact info
    if(grasp_open_close_timestamp + timeout_duration_steady_ < robot_state_->getTime())
    {
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        fingertip_sensor_second_steady0_[i] = pressure_state_.l_finger_tip[i+num_pressure_pads_side_];
        fingertip_sensor_second_steady1_[i] = pressure_state_.r_finger_tip[i+num_pressure_pads_side_];
      }
      position_second_compression = joint_->position_;
      closed_loop_grasp_state = complete;
      grasp_open_close_timestamp = robot_state_->getTime();
      //compute k
      spring_const = (high_force_-low_force_)/(position_second_compression - position_first_compression);

      //publish finding
      //if the grasp was of an unknown object, and want to identify it
      int peak_sum = 0;
      int steady_sum = 0;
      std::ofstream myfile;
      myfile.open("grasp_data.txt");
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        peak_sum += fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i];
        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_peak1_[i] - fingertip_sensor_start1_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {
        steady_sum += fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_first_steady1_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_peak0_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_peak1_[i] - fingertip_sensor_start1_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_steady0_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      for(int i = 0; i < num_pressure_pads_front_; i++)
      {

        if(i%3 == 0)
        {
          myfile << "\n";
        }
        myfile << fingertip_sensor_second_steady1_[i] - fingertip_sensor_start0_[i] << " ";
      }
      myfile << "\n";
      myfile << "\n";
      myfile << spring_const << "\n" << position_first_contact << "\n" <<position_first_compression << "\n" << position_second_contact << "\n" << position_second_compression;
      myfile << "\n";
      myfile << "\n";
      myfile << peak_sum << "\n" << steady_sum;
      myfile.close();

      if(closed_loop)
      {
        service_response_.distance = position_first_contact;
        service_response_.effort = -1.0*high_force_;
        int force_peak0 = 0;
        int force_steady0 = 0;
        int force_peak1 = 0;
        int force_steady1 = 0;
        //TODO::some of this can be done above...
        for(int i = 0; i < num_pressure_pads_front_; i++)
        {
          service_response_.fingertip_profile0[i+num_pressure_pads_side_] = fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          service_response_.fingertip_profile1[i+num_pressure_pads_side_] = fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
          force_peak0 += fingertip_sensor_first_peak0_[i] - fingertip_sensor_start0_[i];
          force_steady0 += fingertip_sensor_first_steady0_[i] - fingertip_sensor_start0_[i];
          force_peak1 += fingertip_sensor_first_peak1_[i] - fingertip_sensor_start1_[i];
          force_steady1 += fingertip_sensor_first_steady1_[i] - fingertip_sensor_start1_[i];
        }
        for(int i = 0; i < num_pressure_pads_side_; i++)
        {
          service_response_.fingertip_profile0[i] = pressure_state_.l_finger_tip[i]-fingertip_sensor_sides_start0_[i];
          service_response_.fingertip_profile1[i] = pressure_state_.r_finger_tip[i]-fingertip_sensor_sides_start1_[i];
        }
        service_response_.stiffness = spring_const;
        service_response_.force_peak0 = force_peak0;
        service_response_.force_steady0 = force_steady0;
        service_response_.force_peak1 = force_peak1;
        service_response_.force_steady1 = force_steady1;
        service_response_.distance_compressed_first = position_first_compression;
        service_response_.distance_compressed_second = position_second_compression;
        service_success_ = true;
      }
      ROS_INFO("Grasp complete!");
    }
    velocity_mode_ = false;
    return -1.0*high_force_;
  }
*/
  //finishing the grasp
  else if(closed_loop_grasp_state == complete)
  {
    velocity_mode_ = false;
    service_flag_ = true;
    if(service_callback)
    {
      if(closed_loop)
        return service_request_.effort;
      return -1.0*(force_+force_increase_*(closed_loop_trail_-1));
    }
    //TODO::determine a good output force
    return -1.0*(force_);  //because we're done
  }

  else if(closed_loop_grasp_state == failed)
  {
    velocity_mode_ = false;
    service_flag_ = true;
    return default_effort_;
  }
  velocity_mode_ = false;
  return 0.0;

}

double Pr2GripperControllerOld::parseMessage(experimental_controllers::GripperControllerCmd desired_msg)
{
  if(desired_msg.cmd.compare("grasp_cl") == 0)
  {
    return grasp(service_callback_, true);
  }
  else if(desired_msg.cmd.compare("grasp") == 0)
  {
    return grasp(service_callback_, false);
  }
  else if(desired_msg.cmd.compare("move") == 0)
  {
    velocity_mode_ = false;
    return desired_msg.val;
  }
  else if(desired_msg.cmd.compare("moveVel") == 0)
  {
    velocity_mode_ = true;
    return desired_msg.val;
  }
  else if(desired_msg.cmd.compare("moveTo") == 0)
  {
    velocity_mode_ = false;
    double direction = 1.0;
    if(joint_->position_ > desired_msg.val)
      direction = -1.0;
    if(proportional_offset_ > fabs(joint_->position_-desired_msg.val))
      return default_effort_*direction*fabs(joint_->position_-desired_msg.val)/proportional_offset_;
    return default_effort_*direction;
  }
  //This is calculated elsewhere
  else if(desired_msg.cmd.compare("step") == 0)
  {
    velocity_mode_ = false;
    return last_commanded_command;
  }
  else if(desired_msg.cmd.compare("ramp") == 0)
  {
    velocity_mode_ = false;
    return rampMove(desired_msg.start, desired_msg.end, desired_msg.time, desired_msg.end);
  }
  else if(desired_msg.cmd.compare("open") == 0)
  {
    velocity_mode_ = false;
    return default_effort_;
  }
  else if(desired_msg.cmd.compare("close") == 0)
  {
    velocity_mode_ = false;
    return -1.0*default_effort_;
  }
  else
  {
    velocity_mode_ = false;
    return 0.0;
  }
}

double Pr2GripperControllerOld::effortLimit(double desiredEffort)
{
  if(desiredEffort > joint_->joint_->limits->effort)
    return joint_->joint_->limits->effort;
  if(desiredEffort < joint_->joint_->limits->effort*-1.0)
    return joint_->joint_->limits->effort*-1.0;
  return desiredEffort;
}

void Pr2GripperControllerOld::command_callback(const experimental_controllers::GripperControllerCmdConstPtr& grasp_cmd)
{
  pthread_mutex_lock(&pr2_gripper_controller_lock_);
  grasp_cmd_.cmd = grasp_cmd->cmd;
  grasp_cmd_.start = grasp_cmd->start;
  grasp_cmd_.end = grasp_cmd->end;
  grasp_cmd_.time = grasp_cmd->time;
  grasp_cmd_.val = grasp_cmd->val;
  closed_loop_trail_ = 0;
  cmd_received_timestamp_ = robot_state_->getTime();
  new_cmd_available_ = true;
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
}

void Pr2GripperControllerOld::pressure_state_callback(const pr2_msgs::PressureStateConstPtr& pressure_state)
{
  if((int)pressure_state->l_finger_tip.size() != num_pressure_pads_front_+num_pressure_pads_side_)
    ROS_WARN("fingertip pressure data is different size than expected");
  pthread_mutex_lock(&pr2_gripper_controller_lock_);
  for(int i = 0; i < num_pressure_pads_front_+num_pressure_pads_side_; i++)
  {
    pressure_state_.l_finger_tip[i] = pressure_state->l_finger_tip[i];
    pressure_state_.r_finger_tip[i] = pressure_state->r_finger_tip[i];
  }
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
}

bool Pr2GripperControllerOld::grasp_cl_srv(experimental_controllers::GraspClosedLoop::Request &req, experimental_controllers::GraspClosedLoop::Response &res)
{
  pthread_mutex_lock(&pr2_gripper_controller_lock_);
  grasp_cmd_.cmd = req.cmd;
  service_request_ = req;
  new_cmd_available_ = true;
  closed_loop_grasp_state = unstarted;
  closed_loop_trail_ = 0;
  if(req.trials < 1)
    return false;
  cmd_received_timestamp_ = robot_state_->getTime();
  service_response_.fingertip_profile0.resize(num_pressure_pads_front_+num_pressure_pads_side_);
  service_response_.fingertip_profile1.resize(num_pressure_pads_front_+num_pressure_pads_side_);
  service_response_.distance.resize(req.trials);
  service_response_.effort.resize(req.trials);
  service_response_.stiffness.resize(req.trials);
  service_response_.force_peak0.resize(req.trials);
  service_response_.force_steady0.resize(req.trials);
  service_response_.force_peak1.resize(req.trials);
  service_response_.force_steady1.resize(req.trials);
  service_response_.distance_compressed.resize(req.trials);
  service_response_.distance_peak.resize(req.trials);
  service_response_.time.resize(req.trials);
  service_response_.time_to_first.resize(req.trials-1);
  service_response_.time_to_return.resize(req.trials-1);
  for(int i = 0; i < req.trials - 1; i++)
    service_response_.time_to_first[i] = -1.0;
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
  service_flag_ = false;
  service_success_ = false;
  //TODO::check if this is thread safe
  service_callback_ = true;
  while(!service_flag_)
  {
    usleep(50000);
  }
  res = service_response_;
  //TODO::check if this is thread safe
  service_callback_ = false;
  return service_success_;
}

void Pr2GripperControllerOld::callbackThread()
{
  //ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  while (node_.ok())
  {
    service_queue_.callAvailable(ros::WallDuration(0.01));
  }
}
