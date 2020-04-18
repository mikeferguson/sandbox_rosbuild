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

#ifndef CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H_
#define CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H_

// system includes
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

// ros includes
#include <ros/ros.h>
#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>
// #include <robot_mechanism_controllers/joint_effort_controller.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "kdl/chainfksolverpos_recursive.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Core>

#include <geometry_msgs/Twist.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/chain.h>
#include <tf/transform_datatypes.h>

#include <filters/transfer_function.h>

// local includes
#include <experimental_controllers/joint_velocity_filtered_controller.h>
#include <experimental_controllers/JointPositionVelocityStamped.h>
#include <experimental_controllers/JointPositionVelocityStampedArray.h>
#include <experimental_controllers/PoseTwistStamped.h>
#include <experimental_controllers/PoseTwistStampedArray.h>
#include <experimental_controllers/NullspaceTermStamped.h>
#include <experimental_controllers/NullspaceTermStampedArray.h>
// #include <experimental_controllers/ErrorTermStamped.h>
// #include <experimental_controllers/ErrorTermStampedArray.h>

namespace controller
{

class CartesianTwistControllerIkWithNullspaceOptimization: public pr2_controller_interface::Controller
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	/*!
	 */
	CartesianTwistControllerIkWithNullspaceOptimization();
	~CartesianTwistControllerIkWithNullspaceOptimization();

	/*!
	 * @param robot_state
	 * @param node_handle
	 * @return
	 */
	bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node_handle);

	/*!
	 * @return
	 */
	void starting();

	/*!
	 */
	void update();

	/*!
	 */
	void stopping();

	/*! input of the cartesian twist controller
	 */
	KDL::Frame kdl_pose_desired_;
	KDL::Twist kdl_twist_desired_;

	/*! output
	 */
	KDL::Frame kdl_pose_measured_;
	KDL::Twist kdl_twist_measured_;

	/*! only for debugging...
	 */
	KDL::JntArray kdl_current_joint_positions_;
	KDL::JntArrayVel kdl_current_joint_velocities_;

	/*! input to the controller for the nullspace optimization part
	 */
	Eigen::Matrix<double, 7, 1> rest_posture_joint_configuration_;

private:

	/*! robot description
	 */
	pr2_mechanism_model::RobotState *robot_state_;
	pr2_mechanism_model::Chain mechanism_chain_;


	/*!
	 */
	ros::NodeHandle node_handle_;

	/*!
	 * @param quat1
	 * @param quat2
	 * @param angular_velocity_error
	 */
	void computeAngularVelocityError(const double* quat1, const double* quat2, double* angular_velocity_error);
	void getQuaternionFromRPY(const double roll, const double pitch, const double yaw, double &qx, double &qy, double &qz, double &qw);

	/*!
	 */
	KDL::Twist kdl_twist_error_;

	/*!
	 */
// 	boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
// 	boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
// 	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
	boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
	boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

	/*!
	 */
	double nullspace_weight_;

	/*!
	 */
	ros::Time last_time_;

	/*!
	 */
	double ff_trans_;
	double ff_rot_;

	/*! feedback pid controllers (translation and rotation)
	 */
	std::vector<control_toolbox::Pid> cartesian_fb_pid_controllers_;

	/*! feedback pid controller (nullspace)
	 */
	std::vector<control_toolbox::Pid> nullspace_fb_pid_controllers_;

	/*! kdl stuff for kinematics
	 */
	KDL::Chain kdl_chain_;

	/*!
	 */
	KDL::Jacobian kdl_chain_jacobian_;

	/*!
	 */
	Eigen::MatrixXd eigen_chain_jacobian_;

	Eigen::Matrix<double, 6, 6> eigen_jac_times_jac_transpose_;
	Eigen::Matrix<double, 6, 6> eigen_jjt_inverse_;

	Eigen::Matrix<double, 7, 7> eigen_nullspace_projector_;
	Eigen::Matrix<double, 7, 7> eigen_identity_;

	Eigen::Matrix<double, 6, 1> eigen_desired_cartesian_velocities_;
	Eigen::Matrix<double, 7, 1> eigen_desired_joint_velocities_;

	Eigen::Matrix<double, 7, 1> eigen_nullspace_error_;
	Eigen::Matrix<double, 7, 1> eigen_nullspace_term_;

	Eigen::MatrixXd eigen_jac_pseudo_inverse_;

	/*!
	 */
	// std::vector<boost::shared_ptr<JointVelocityFilteredController> > joint_velocity_controllers_;
	std::vector<JointVelocityFilteredController*> joint_velocity_controllers_;
	//	std::vector<JointEffortController*> joint_effort_controllers_;

	/*!
	 */
	int num_joints_;

	/*!
	 */
  filters::MultiChannelTransferFunctionFilter<double> pose_filter_;
  std::vector<double> pose_unfiltered_data_;
  std::vector<double> pose_filtered_data_;

  /*!
   */
  filters::MultiChannelTransferFunctionFilter<double> joint_positions_filter_;
  std::vector<double> joint_positions_filter_values_;

  /*!
   */
  int publisher_buffer_size_;

  int desired_joint_buffer_counter_;
  int actual_joint_buffer_counter_;

  experimental_controllers::JointPositionVelocityStampedArray joint_desired_array_;
  realtime_tools::RealtimePublisher<experimental_controllers::JointPositionVelocityStampedArray> *joint_desired_array_publisher_;
  experimental_controllers::JointPositionVelocityStampedArray joint_actual_array_;
  realtime_tools::RealtimePublisher<experimental_controllers::JointPositionVelocityStampedArray> *joint_actual_array_publisher_;

  int desired_pose_twist_buffer_counter_;
  int actual_pose_twist_buffer_counter_;

  experimental_controllers::PoseTwistStampedArray pose_twist_desired_array_;
  realtime_tools::RealtimePublisher<experimental_controllers::PoseTwistStampedArray> *pose_twist_desired_array_publisher_;
  experimental_controllers::PoseTwistStampedArray pose_twist_actual_array_;
  realtime_tools::RealtimePublisher<experimental_controllers::PoseTwistStampedArray> *pose_twist_actual_array_publisher_;

  int nullspace_term_buffer_counter_;
  experimental_controllers::NullspaceTermStampedArray nullspace_term_array_;
  realtime_tools::RealtimePublisher<experimental_controllers::NullspaceTermStampedArray> *nullspace_term_array_publisher_;

  /*!
   */
  void aggregateAndPublish();

	/*!
	 */
	bool keep_endeffector_fixed_for_testing_;
	KDL::Frame kdl_pose_fixed_;

};

// inline void CartesianTwistControllerIkWithNullspaceOptimization::setRestPosture(double *rest_posture)
// {
// 	for (int i = 0; i < 7; i++)
// 	{
// 		rest_posture_joint_configuration_(i) = rest_posture[i];
// 	}
// // 	rest_posture_is_set_ = true;
// }

} // namespace

#endif /* CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H_ */
