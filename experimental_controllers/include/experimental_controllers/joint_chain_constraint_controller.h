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


/***************************************************/
/*! \class controller::JointChainConstraintController
    \brief Adds joint position constraints to any connected chain of joints.

    Wrenches are in the cartesian frame in the root frame on the tip frame.

    Example config:<br>

    <controller type="JointChainConstraintController" name="controller_name"><br>
       <chain root="torso_lift_link" tip="r_gripper_tool_frame" offset="0 0 0" /><br>
    </controller><br>
*/
/***************************************************/


#ifndef JOINT_CHAIN_CONSTRAINT_CONTROLLER_H
#define JOINT_CHAIN_CONSTRAINT_CONTROLLER_H

#include <vector>
#include <cstdio>
#include <cstring>
#include <string>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "geometry_msgs/Wrench.h"
#include "experimental_controllers/ChangeConstraints.h"
#include "experimental_controllers/JointConstraint.h"
#include "pr2_controller_interface/controller.h"
#include "pr2_mechanism_model/chain.h"

#include "tf/transform_datatypes.h"
//#include "misc_utils/advertised_service_guard.h"

#include "Eigen/Geometry"
#include "Eigen/LU"
#include "Eigen/Core"


namespace controller {

class ConstraintState
{
public:
  std::string id_;
  std::string joint_name_;
  double threshold_start_;
  double nullspace_start_;
  double max_constraint_torque_;
  pr2_mechanism_model::Joint* joint_;
  int joint_chain_index_;
  double joint_error_;
  bool remove_;

  control_toolbox::Pid pid_;
};


class JointChainConstraintController
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  JointChainConstraintController();
  ~JointChainConstraintController();

  /*!
   * \brief Functional way to initialize the chain and construct jacobians
   * \param root_name The root of the chain.
   * \param tip_name The tip of the chain.
   * \param *robot The robot.
   */
  bool init(pr2_mechanism_model::RobotState *robot, const std::string& root_name,
            const std::string& tip_name, const std::string& controller_name);

  void update();

  // input of the controller
  KDL::Wrench wrench_desired_;


private:
  void computeConstraintTorques();
  void computeConstraintJacobian();
  void computeConstraintNullSpace();
  bool addConstraint(experimental_controllers::ChangeConstraints::Request &req,
                     experimental_controllers::ChangeConstraints::Response &resp);

  ros::NodeHandle node_;
  Eigen::Matrix<float,6,1> task_wrench_;
  std::list<ConstraintState> constraint_list_;
  int list_size_;
  pr2_mechanism_model::Chain mechanism_chain_;

  std::string controller_name_;
  pr2_mechanism_model::RobotState *robot_state_;

  KDL::Chain kdl_chain_;
  KDL::ChainJntToJacSolver *jnt_to_jac_solver_;
  KDL::JntArray jnt_pos_, jnt_eff_;
  KDL::Jacobian chain_kdl_jacobian_;

  Eigen::MatrixXf identity_;
  Eigen::MatrixXf chain_eigen_jacobian_;
  Eigen::MatrixXf task_torque_;

  // joint constraint matrices and vectors
  Eigen::MatrixXf joint_constraint_torque_;
  Eigen::MatrixXf joint_constraint_jacobian_;
  Eigen::MatrixXf joint_constraint_null_space_;

  ros::Time last_time_;

  bool initialized_;

  //  AdvertisedServiceGuard change_constraints_guard_;

  ros::ServiceServer add_constraints_service_;

};



/*
class JointChainConstraintControllerNode : public pr2_controller_interface::Controller
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  JointChainConstraintControllerNode();
  ~JointChainConstraintControllerNode();

  bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);
  void update();

 private:
  void command();

  ros::Node* node_;
  std::string controller_name_;
  JointChainConstraintController controller_;
  pr2_mechanism_model::RobotState *robot_;

  geometry_msgs::Wrench wrench_msg_;


};
*/
} // namespace


#endif
