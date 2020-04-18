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


#ifndef JOINT_VELOCITY_FILTERED_CONTROLLER_TUNER_H_
#define JOINT_VELOCITY_FILTERED_CONTROLLER_TUNER_H_

#include <ros/ros.h>
#include <pr2_controller_interface/controller.h>
#include <experimental_controllers/joint_velocity_filtered_controller.h>
#include <std_msgs/Float64.h>

namespace controller
{

class JointVelocityFilteredControllerTuner : public pr2_controller_interface::Controller
{
public:

	/*!
	 * @return
	 */
  JointVelocityFilteredControllerTuner();
  ~JointVelocityFilteredControllerTuner();

  /*!
   * @param robot
   * @param config
   * @return
   */
  bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);
  bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node_handle);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param double pos Velocity command to issue
   */
  void setCommand(double cmd);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
  void getCommand(double &cmd);

  /*!
   * @return
   */
  void starting();
  void update();

  /*!
   * @param p
   * @param i
   * @param d
   * @param i_max
   * @param i_min
   */
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

  /*!
   * @return
   */
  std::string getJointName();

private:

  bool initialized_;
  JointVelocityFilteredController *joint_velocity_filtered_controller_;

  ros::NodeHandle node_handle_;
  pr2_mechanism_model::RobotState *robot_state_;

  double sine_frequency_;
  double sine_amplitude_;
  double sine_offset_;

  ros::Time last_time_;
  ros::Time time_;
  double actual_time_;

  double sine_wave_value_;

};

} // namespace

#endif /* JOINT_VELOCITY_FILTERED_CONTROLLER_TUNER_H_ */
