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

#include <ros/ros.h>

#include <pr2_controller_interface/controller.h>

#include <manipulation_msgs/Waypoint.h>

#include <manipulation_msgs/SplineTraj.h>
#include <manipulation_msgs/SplineTrajSegment.h>

#include <manipulation_srvs/SetSplineTraj.h>
#include <manipulation_srvs/QuerySplineTraj.h>
#include <manipulation_srvs/CancelSplineTraj.h>

#include <realtime_tools/realtime_publisher.h>

#include <experimental_controllers/pid_position_velocity_controller.h>

#include <boost/shared_ptr.hpp>

namespace controller {

  class TrajectoryController : public pr2_controller_interface::Controller
  {
    public:

    TrajectoryController(); /*** Constructor ***/

    ~TrajectoryController(); /*** Destructor ***/

    bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &n);

    void starting();

    void update();

    bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);

    private:

    ros::NodeHandle node_;

    std::string controller_name_;

    // robot structure
    pr2_mechanism_model::RobotState *robot_state_;

    // pid controllers

    std::vector<boost::shared_ptr<controller::PIDPositionVelocityController> > joint_controllers_;

    // reatltime publisher
//    boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > state_error_publisher_;

//    boost::scoped_ptr<realtime_tools::RealtimePublisher<> > spline_info_publisher_;

//    boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > state_pose_publisher_;

    int spline_num_segments_, num_joints_, spline_index_, trajectory_id_;

    bool new_cmd_available_, spline_done_;

    ros::Time last_time_;
    ros::Duration spline_time_;

    manipulation_msgs::Waypoint getCommand(const manipulation_msgs::SplineTrajSegment &spline, const double t);

    manipulation_msgs::SplineTraj spline_traj_;

    manipulation_msgs::SplineTraj setCmdToCurrent(void);

    bool setSplineTraj(manipulation_srvs::SetSplineTraj::Request &req,
                       manipulation_srvs::SetSplineTraj::Response &resp);

    bool querySplineTraj(manipulation_srvs::QuerySplineTraj::Request &req,
                                           manipulation_srvs::QuerySplineTraj::Response &resp);

    bool cancelSplineTraj(manipulation_srvs::CancelSplineTraj::Request &req,
                                            manipulation_srvs::CancelSplineTraj::Response &resp);

    void setCommand(const manipulation_msgs::Waypoint &wp);

    std::vector<std::string> joint_names_;

    manipulation_msgs::SplineTraj spline_cmd_, spline_rt_;

    ros::ServiceServer trajectory_set_service_;

    ros::ServiceServer trajectory_query_service_;

    ros::ServiceServer trajectory_cancel_service_;

    /*!
     * \brief mutex lock for setting and getting commands
     */
    pthread_mutex_t spline_cmd_lock_;

    void resetControllers();

    int trajectory_status_;

    manipulation_msgs::Waypoint goal_;

    std::vector<double> goal_reached_threshold_;

    bool goalReached();

    manipulation_msgs::Waypoint cmd_;

    manipulation_msgs::SplineTraj current_position_cmd_;

    int num_controllers_;
  };
}
