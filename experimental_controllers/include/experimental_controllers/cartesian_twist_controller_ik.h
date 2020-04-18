/*
 * Copyright (c) 2008, Ruben Smits and Willow Garage, Inc.
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

/*
 * Author: Wim Meeussen
 *         Ruben Smits
 */

#ifndef CARTESIAN_TWIST_CONTROLLER_IK_H
#define CARTESIAN_TWIST_CONTROLLER_IK_H

#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <tf/transform_datatypes.h>
#include <control_toolbox/pid.h>
#include <experimental_controllers/joint_inverse_dynamics_controller.h>

namespace controller
{

class CartesianTwistControllerIk: public pr2_controller_interface::Controller
{
public:
	CartesianTwistControllerIk();
	~CartesianTwistControllerIk();

	bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle& n);
	void starting();
	void update();

	void command(const geometry_msgs::TwistConstPtr& twist_msg);

	// input of the controller
	KDL::Twist twist_desi_;

private:
	KDL::Twist twist_meas_, error, twist_out_;

	ros::NodeHandle node_;
	ros::Subscriber sub_command_;

	ros::Time last_time_;
	double ff_trans_, ff_rot_;

	// pid controllers
	std::vector<control_toolbox::Pid> fb_pid_controller_;

	// robot description
	pr2_mechanism_model::RobotState *robot_state_;
	pr2_mechanism_model::Chain chain_;

	// kdl stuff for kinematics
	KDL::Chain kdl_chain_;
	boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
	boost::scoped_ptr<KDL::ChainIkSolverVel> twist_to_jnt_solver_;
	KDL::JntArrayVel jnt_posvel_;

	JointInverseDynamicsController* id_controller_;
};

} // namespace


#endif
