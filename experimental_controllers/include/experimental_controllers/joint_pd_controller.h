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

#pragma once

/***************************************************/
/*! \class controller::JointPDController
  \brief Joint Velocity Controller

  This class closes the loop around velocity using
  a pid loop.

  Example config:<br>

  <controller type="JointPDController" name="controller_name"><br>
    <joint name="joint_to_control"><br>
      <pid p="1.0" i="2.0" d="3.0" iClamp="4.0" /><br>
    </joint>
  </controller>
*/
/***************************************************/

#include <ros/ros.h>

#include <control_toolbox/pid.h>
//#include "misc_utils/advertised_service_guard.h"
//#include "misc_utils/subscription_guard.h"
#include <pr2_mechanism_model/robot.h>

// Services
#include <experimental_controllers/SetPDCommand.h>
#include <experimental_controllers/GetPDCommand.h>
#include <deprecated_msgs/JointCmd.h>

namespace controller
{

  class JointPDController 
  {
    public:

    JointPDController();
    ~JointPDController();

    /*!
     * \brief Functional way to initialize limits and gains.
     * \param pid Pid gain values.
     * \param joint_name Name of joint we want to control.
     * \param *robot The robot.
     */
     bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);
     bool init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,const control_toolbox::Pid &pid);

    /*!
     * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
     *
     * \param double pos Velocity command to issue
     */
    void setCommand(double command, double command_dot);

    /*!
     * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
     */
    void getCommand(deprecated_msgs::JointCmd & cmd);


    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

    std::string getJointName();
    pr2_mechanism_model::JointState *joint_state_;        /**< Joint we're controlling. */

    /*!
     * \brief Reset the internal PID controllers
     */
    void reset();

  private:

    pr2_mechanism_model::RobotState *robot_;             /**< Pointer to robot structure. */
    control_toolbox::Pid pid_controller_;      /**< Internal PID controller. */
    ros::Time last_time_;                         /**< Last time stamp of update. */
    double command_;                           /**< Last commanded position. */
    double command_dot_;
    double command_t_;                         /**< Last commanded position. */
    double command_dot_t_;

    /*!
     * \brief mutex lock for setting and getting commands
     */
    pthread_mutex_t joint_pd_controller_lock_;

  };

/***************************************************/
/*! \class controller::JointPDControllerNode
  \brief Joint PD Controller ROS Node

  This class closes the loop around velocity using
  a pid loop.

*/
/***************************************************/

  class JointPDControllerNode 
  {
    public:

    JointPDControllerNode();
    ~JointPDControllerNode();

    void update();
    bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);

    // Topics
    void setCommand(const deprecated_msgs::JointCmdConstPtr &msg);


    private:

    //node stuff
    std::string service_prefix_;                 /**< The name of the controller. */
    ros::NodeHandle node_;
    //    SubscriptionGuard guard_set_command_;        /**< Makes sure the subscription goes down neatly. */

    //msgs
    deprecated_msgs::JointCmd cmd_;                      /**< The command from the subscription. */

    //controller
    JointPDController *c_;                       /**< The controller. */
    ros::Subscriber cmd_subscriber_;
  };
}
