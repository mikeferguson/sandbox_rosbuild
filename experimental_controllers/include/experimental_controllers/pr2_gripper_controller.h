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

#include <ros/ros.h>
#include <pr2_mechanism_model/robot.h>
#include <pr2_controller_interface/controller.h>
#include <robot_mechanism_controllers/joint_effort_controller.h>
#include <experimental_controllers/GripperControllerCmd.h>
#include <experimental_controllers/GripperControllerState.h>
#include <pr2_msgs/PressureState.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_toolbox/pid.h>
#include <experimental_controllers/GraspClosedLoop.h>
#include <boost/thread.hpp>
#include "ros/callback_queue.h"

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

namespace controller
{
  class Pr2GripperController: public pr2_controller_interface::Controller
  {
    public:
      enum grasp_state {unstarted, open0, close0_closing, close0_contact, open1, close1_closing, close1_contact, complete, failed};

      Pr2GripperController();

      ~Pr2GripperController();

      /*!
       * \brief Loads controller's information from the xml description file and param server
       * @param robot_state The robot's current state
       * @param config Tiny xml element pointing to this controller
       * @return Successful init
       */
      bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node);

      bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);

      /*!
       * \brief (a) Updates commands to the gripper.
       * Called every timestep in realtime
       */
      void update();

      void starting();

      void stopping();

      double rampMove(double start_force, double end_force, double time, double hold);

      double stepMove(double step_size);

      double grasp(bool service_callback, bool closed_loop);

      void callbackThread();

    private:


      std::string name_;

      std::string fingertip_sensor_topic_;

      double default_effort_;

      double default_low_speed_;

      double default_high_speed_;

      void command_callback(const experimental_controllers::GripperControllerCmdConstPtr& grasp_cmd);

      void pressure_state_callback(const pr2_msgs::PressureStateConstPtr& pressure_state);

      bool grasp_cl_srv(experimental_controllers::GraspClosedLoop::Request &req, experimental_controllers::GraspClosedLoop::Response &res);

      experimental_controllers::GripperControllerCmd grasp_cmd_desired_;

      experimental_controllers::GripperControllerCmd grasp_cmd_;

      //pr2_mechanism_controllers::GripperControllerCmd grasp_cmd;

      pr2_msgs::PressureState pressure_state_;

      //ethercat_hardware::PressureState pressure_state;

      /*!
       * \brief mutex lock for setting and getting commands
       */
      pthread_mutex_t pr2_gripper_controller_lock_;

      /*!
       * \brief true when new command received by node
       */
      bool new_cmd_available_;

      /*!
       * \brief timeout specifying time that the controller waits before setting the current velocity command to zero
       */
      double timeout_;

      /*!
       * \brief time corresponding to when update was last called
       */
      ros::Time last_time_;

      /*!
       * \brief time corresponding to last publish
       */
      ros::Time last_published_time_;

      /*!
       * \brief rate at which states are published
       */
      double publish_rate_;

      /*!
      * \brief timestamp remembering when the last command was received
      */
      ros::Time cmd_received_timestamp_;

      /*!
      * \brief amplitude in joint effort values used to break stiction
      */
      double break_stiction_amplitude_;

      /*!
       * \brief duration in seconds used to increment effort or period of effort's overlaid sine wave
       */
      double break_stiction_period_;

      /*!
       * \brief velocity at which stiction is consitered broken
       */
      double break_stiction_velocity_;

      /*!
       * \brief type of stiction breaking: none, sine, ramp
       */
      std::string break_stiction_type_;

      /*!
       * \brief last commanded command stripped of any break stiction forces
       */
      double last_commanded_command;

      /*!
       * \brief offset from desired value where the moveTo command goes from full force to linear
       */
      double proportional_offset_;

      /*!
       * \brief remembers last impact time
       */
      double grasp_impact_timestamp;

      /*!
       * \brief remembers last time it was commanded to close or open
       */
      ros::Time grasp_open_close_timestamp;

      /*!
       * \brief remembers last time it was commanded to close or open
       */
      double timeout_duration_;

      /*!
      * \brief time after the object has been sqeezed that can be considered steady
      */
      double timeout_duration_steady_;

      /*!
       * \brief remembers the state of the closed loop grasp
       */
      grasp_state closed_loop_grasp_state;

      /*!
       * \brief remembers everything about the state of the robot
       */
      pr2_mechanism_model::RobotState *robot_state_;

      /*!
       * \brief JointState for this caster joint
       */
      pr2_mechanism_model::JointState *joint_;

      JointEffortController joint_controller_;

      /*!
       * \brief The maximum value that can be counted as stopped for closed loop grasping
       */
      double stopped_threshold_;

      double parseMessage(experimental_controllers::GripperControllerCmd desired_msg);

      double effortLimit(double desiredEffort);

      /*!
       * \brief publishes information about the caster and wheel controllers
       */
      boost::scoped_ptr<realtime_tools::RealtimePublisher<experimental_controllers::GripperControllerState> > state_publisher_;

      std::vector<int> fingertip_sensor_start0_;
      std::vector<int> fingertip_sensor_start1_;
      std::vector<int> fingertip_sensor_first_peak0_;
      std::vector<int> fingertip_sensor_first_peak1_;
      std::vector<int> fingertip_sensor_first_steady0_;
      std::vector<int> fingertip_sensor_first_steady1_;
      std::vector<int> fingertip_sensor_second_peak0_;
      std::vector<int> fingertip_sensor_second_peak1_;
      std::vector<int> fingertip_sensor_second_steady0_;
      std::vector<int> fingertip_sensor_second_steady1_;
      std::vector<int> fingertip_sensor_sides_start0_;
      std::vector<int> fingertip_sensor_sides_start1_;

      double position_first_contact;
      //double position_second_contact;
      double position_first_compression;
      //double position_second_compression;
      int peak_force_first_grasp;
      //int peak_force_second_grasp;
      double peak_distance_first_grasp;
      //double peak_distance_second_grasp;

      double force_;
      double force_increase_;

      double spring_const;

      int contact_threshold_;

      int contact_threshold_individual_;

      bool service_flag_;

      bool service_success_;

      /*
       * \brief remembers if the current grasp command is from a service or a topic
       */
      bool service_callback_;

      experimental_controllers::GraspClosedLoop::Request service_request_;
      experimental_controllers::GraspClosedLoop::Response service_response_;

      int num_pressure_pads_side_;
      int num_pressure_pads_front_;

      control_toolbox::Pid p_i_d_;

      /*
       * \brief remembers if the control scheme is velocity control or effort control
       */
      bool velocity_mode_;

      ros::Time last_velocity_time_;

      ros::CallbackQueue service_queue_;

      boost::thread service_thread_;

      ros::Subscriber cmd_sub_;

      ros::Subscriber pressure_sub_;

      ros::NodeHandle node_;

      ros::ServiceServer grasp_service_;

      int closed_loop_trail_;
  };
}

