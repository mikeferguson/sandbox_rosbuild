/*
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
 * Author: Melonee Wise
 */

#ifndef ENDEFFECTOR_CONSTRAINT_CONTROLLER_H
#define ENDEFFECTOR_CONSTRAINT_CONTROLLER_H

#include <vector>
#include "boost/scoped_ptr.hpp"
#include "pr2_mechanism_model/chain.h"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "ros/node.h"
#include "geometry_msgs/Wrench.h"
#include "misc_utils/subscription_guard.h"
#include "pr2_controller_interface/controller.h"
#include "tf/transform_datatypes.h"
#include "misc_utils/advertised_service_guard.h"
#include "joy/Joy.h"
#include "Eigen/LU"
#include "Eigen/Core"
#include <visualization_msgs/Marker.h>


namespace controller {

class EndeffectorConstraintController : public pr2_controller_interface::Controller
{
public:
  EndeffectorConstraintController();
  ~EndeffectorConstraintController();

  bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);
  void update();
  void computeConstraintJacobian();
  void computeConstraintNullSpace();
  // input of the controller
  KDL::Wrench wrench_desi_;
  Eigen::Matrix<float,6,1> task_wrench_;

private:

  pr2_mechanism_model::RobotState *robot_;

  // kdl stuff for kinematics
  pr2_mechanism_model::Chain chain_;
  KDL::Chain kdl_chain_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;

  // to get joint positions, velocities, and to set joint torques
  Eigen::Matrix<float,6,5> constraint_jac_;
  Eigen::Matrix<float,6,1> constraint_wrench_;
  Eigen::Matrix<float,5,1> constraint_force_;
  // joint constraint
  Eigen::MatrixXf joint_constraint_force_;
  Eigen::MatrixXf joint_constraint_jac_;
  Eigen::MatrixXf joint_constraint_null_space_;

  Eigen::MatrixXf task_jac_;
  Eigen::MatrixXf identity_;
  Eigen::MatrixXf identity_joint_;
  Eigen::MatrixXf constraint_null_space_;
  Eigen::MatrixXf constraint_torq_;
  Eigen::MatrixXf joint_constraint_torq_;
  Eigen::MatrixXf task_torq_;
  KDL::Frame endeffector_frame_;
  KDL::Frame desired_frame_;

  // some parameters to define the constraint
  double wall_x;
  double elbow_limit;
  double threshold_x;
  double wall_r;
  double threshold_r;
  double f_x_max;
  double f_r_max;
  double f_pose_max;
  double f_limit_max;

  double desired_roll_;
  double desired_pitch_;
  double desired_yaw_;
  bool initialized_;
};


class EndeffectorConstraintControllerNode : public pr2_controller_interface::Controller
{
 public:
  EndeffectorConstraintControllerNode();
  ~EndeffectorConstraintControllerNode();

  bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);
  void update();
  void command();

 private:
  std::string topic_;
  ros::Node *node_;
  EndeffectorConstraintController controller_;
  SubscriptionGuard guard_command_;

  geometry_msgs::Wrench wrench_msg_;

  unsigned int loop_count_;
};

} // namespace


#endif
