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

// system includes
#include <algorithm>
#include <sstream>

// ros includes
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/kinfam_io.hpp>
#include <pluginlib/class_list_macros.h>

#include <angles/angles.h>

// TODO: remove stdio.h
#include <stdio.h>

// local includes
#include <experimental_controllers/cartesian_twist_controller_ik_with_nullspace_optimization.h>

PLUGINLIB_REGISTER_CLASS(CartesianTwistControllerIkWithNullspaceOptimization, controller::CartesianTwistControllerIkWithNullspaceOptimization, pr2_controller_interface::Controller);

namespace controller
{

const int CONTINOUS_FOREARM_ROLL_JOINT = 4;
const int CONTINOUS_WRIST_ROLL_JOINT = 6;

CartesianTwistControllerIkWithNullspaceOptimization::CartesianTwistControllerIkWithNullspaceOptimization() :
	robot_state_(NULL), jnt_to_twist_solver_(NULL), jnt_to_pose_solver_(NULL), jnt_to_jac_solver_(NULL), num_joints_(0), publisher_buffer_size_(0),
			desired_joint_buffer_counter_(0), actual_joint_buffer_counter_(0), desired_pose_twist_buffer_counter_(0), actual_pose_twist_buffer_counter_(0)
{
}

CartesianTwistControllerIkWithNullspaceOptimization::~CartesianTwistControllerIkWithNullspaceOptimization()
{
	// TODO: delete allocated memory
}

bool CartesianTwistControllerIkWithNullspaceOptimization::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle& node_handle)
{

	assert(robot_state);
	robot_state_ = robot_state;

	node_handle_ = node_handle;

	// get name of root and tip from the parameter server as well as damping and threshold
	std::string root_name, tip_name;
	double damping;
	if (!node_handle_.getParam("root_name", root_name))
	{
		ROS_ERROR("No \"root_name\" found on parameter server");
		return false;
	}
	if (!node_handle_.getParam("tip_name", tip_name))
	{
		ROS_ERROR("No \"tip_name\" found on parameter server");
		return false;
	}
	if (!node_handle_.getParam("damping", damping))
	{
		ROS_ERROR("No \"damping\" parameter found on parameter server");
		return false;
	}
	if (!node_handle_.getParam(std::string("keep_endeffector_fixed_for_testing"), keep_endeffector_fixed_for_testing_))
	{
		ROS_ERROR("No \"keep_endeffector_fixed_for_testing\" parameter found on parameter server.");
		return false;
	}
	if (!node_handle_.getParam("publisher_buffer_size", publisher_buffer_size_))
	{
		ROS_ERROR("No \"publisher_buffer_size\" parameter found on parameter server");
		return false;
	}

	ros::NodeHandle cartesian_ff_gains_handle(std::string("/cartesian_pose_twist_gains"));
	if(!cartesian_ff_gains_handle.getParam(std::string("ff_trans"), ff_trans_))
		{
			ROS_ERROR("No \"cartesian_pose_twist_gains/ff_trans\" parameter found on parameter server");
			return false;
		}
	if(!cartesian_ff_gains_handle.getParam(std::string("ff_rot"), ff_rot_))
		{
			ROS_ERROR("No \"cartesian_pose_twist_gains/ff_rot\" parameter found on parameter server");
			return false;
		}

	std::string restposture_joint_configuration;
	if (!node_handle_.getParam("rest_posture_joint_configuration", restposture_joint_configuration))
	{
		ROS_ERROR("No \"rest_posture_joint_configuration\" parameter found on parameter server");
		return false;
	}

	// create robot chain from root to tip
	if (!mechanism_chain_.init(robot_state, root_name, tip_name))
	{
		return false;
	}
	mechanism_chain_.toKDL(kdl_chain_);
	ROS_INFO("Created kdl chain with %i joints.", kdl_chain_.getNrOfJoints());

	// create forward kinematics solver
	jnt_to_twist_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));

	jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
	kdl_chain_jacobian_.resize(kdl_chain_.getNrOfJoints());
	kdl_current_joint_velocities_.resize(kdl_chain_.getNrOfJoints());

	// create the required matrices
	eigen_identity_.setIdentity();
	eigen_chain_jacobian_ = Eigen::MatrixXd::Zero(6, kdl_chain_.getNrOfJoints());
	eigen_jac_times_jac_transpose_ = Eigen::MatrixXd::Zero(6, 6);
	eigen_jjt_inverse_ = Eigen::MatrixXd::Zero(6, 6);
	eigen_jac_pseudo_inverse_ = Eigen::MatrixXd::Zero(6, kdl_chain_.getNrOfJoints());

	eigen_desired_cartesian_velocities_.setZero(6, 1);
	eigen_nullspace_term_.setZero(7, 1);
	eigen_nullspace_error_.setZero(7, 1);

	// create solver
	jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
	kdl_current_joint_positions_.resize(kdl_chain_.getNrOfJoints());

	rest_posture_joint_configuration_ = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(), 1);

	std::string joint_angle_string;
	// split the controller names based on whitespace
	std::stringstream ss_joint_angles(restposture_joint_configuration);
	unsigned int my_index = 0;
	while (ss_joint_angles >> joint_angle_string)
	{
		double joint_angle;
		std::stringstream ss(joint_angle_string);
		ss >> joint_angle;
		if (my_index < kdl_chain_.getNrOfJoints())
		{
			ROS_INFO("Setting joint %i to %1.3f as rest posture.", my_index, joint_angle);
			rest_posture_joint_configuration_(my_index) = joint_angle;
			my_index++;
		}
		else
		{
			ROS_ERROR("To many joint angles specified in rest posture joint configuration.");
			return false;
		}
	}

	std::string controller_names;
	if (!node_handle_.getParam("velocity_controller_names", controller_names))
	{
		ROS_ERROR("No controller names found on parameter server");
		return false;
	}

	// split the controller names based on whitespace
	std::stringstream ss_controller_names(controller_names);

	num_joints_ = 0;
	std::string controller_name;
	while (ss_controller_names >> controller_name)
	{
		ros::NodeHandle controller_handle(node_handle_, controller_name);

		// boost::shared_ptr<JointVelocityFilteredController> joint_velocity_controller;
		// joint_velocity_controller.reset(new JointVelocityFilteredController());

		JointVelocityFilteredController *joint_velocity_controller = new JointVelocityFilteredController();
		// JointEffortController *joint_effort_controller = new JointEffortController();

		ros::NodeHandle joint_controller_handle(node_handle_, controller_name);
		// if (!joint_effort_controller->init(robot_state_, joint_controller_handle))
		if (!joint_velocity_controller->init(robot_state_, joint_controller_handle))
		{
			ROS_ERROR("Could not initialize controller named %s.", controller_name.c_str());
			return false;
		}

		// joint_effort_controllers_.push_back(joint_effort_controller);
		joint_velocity_controllers_.push_back(joint_velocity_controller);
		num_joints_++;
	}
	// TODO: fix memory leak when JointVelocityFilteredController is constructed but never deleted.

	// constructs 3 identical pid controllers for the x,y and z translations
	control_toolbox::Pid pid_controller;
	if (!pid_controller.init(ros::NodeHandle(node_handle_, "fb_trans")))
	{
		ROS_ERROR("Could not construct pid controller for the x, y, and z translations.");
		return false;
	}
	for (unsigned int i = 0; i < 3; i++)
	{
		cartesian_fb_pid_controllers_.push_back(pid_controller);
	}

	// constructs 3 identical pid controllers for the x,y and z rotations
	if (!pid_controller.init(ros::NodeHandle(node_handle_, "fb_rot")))
	{
		ROS_ERROR("Could not construct pid controller for the x, y, and z rotations.");
		return false;
	}
	for (unsigned int i = 0; i < 3; i++)
	{
		cartesian_fb_pid_controllers_.push_back(pid_controller);
	}

// 	// constructs 7 identical pid controllers for the nullspace
// 	if (!pid_controller.init(ros::NodeHandle(node_handle_, "fb_nullspace")))
// 	{
// 		ROS_ERROR("Could not construct pid controller for the nullspace.");
// 		return false;
// 	}
// 	for (int i = 0; i < num_joints_; i++)
// 	{
// 		nullspace_fb_pid_controllers_.push_back(pid_controller);
// 	}


	std::string nullspace_controller_names;
	if (!node_handle_.getParam(std::string("nullspace_controller_names"), nullspace_controller_names))
	{
		ROS_ERROR_STREAM("Could not get parameter \"nullspace_controller_names\" in namespace " << node_handle_.getNamespace() << ".");
		return false;
	}

	// split the names based on whitespace
	std::stringstream ss_nullspace_controller_names(nullspace_controller_names);
	std::string nullspace_controller_name;
	while (ss_nullspace_controller_names >> nullspace_controller_name)
	{
		if (!pid_controller.init(ros::NodeHandle(node_handle_, nullspace_controller_name)))
			{
				ROS_ERROR("Could not construct pid controller for %s.", nullspace_controller_name.c_str());
				return false;
			}
		nullspace_fb_pid_controllers_.push_back(pid_controller);
	}
	

	pose_filtered_data_.resize(6);
	pose_unfiltered_data_.resize(6);
	if (!((filters::MultiChannelFilterBase<double>&) pose_filter_).configure(6, node_handle_.getNamespace() + std::string("/pose_filter"), node_handle_))
	{
		ROS_ERROR("Could not create velocity filter.");
		return false;
	}

	joint_positions_filter_values_.resize(num_joints_);
	if (!((filters::MultiChannelFilterBase<double>&) joint_positions_filter_).configure(num_joints_, node_handle_.getNamespace() + std::string(
			"/joint_positions_filter"), node_handle_))
	{
		ROS_ERROR("Could not create joint position filter.");
		return false;
	}

	desired_joint_buffer_counter_ = 0;
	actual_joint_buffer_counter_ = 0;
	desired_pose_twist_buffer_counter_ = 0;
	actual_pose_twist_buffer_counter_ = 0;
	nullspace_term_buffer_counter_ = 0;

	joint_desired_array_.array.resize(publisher_buffer_size_);
	joint_actual_array_.array.resize(publisher_buffer_size_);

	// 	joint_desired_array_publisher_.reset(new realtime_tools::RealtimePublisher<experimental_controllers::JointPositionVelocityStampedArray>(node_handle_,
	// 			std::string("joint_position_desired"), 1));
	// 	joint_actual_array_publisher_.reset(new realtime_tools::RealtimePublisher<experimental_controllers::JointPositionVelocityStampedArray>(node_handle_,
	// 			std::string("joint_position_actual"), 1));
	joint_desired_array_publisher_ = new realtime_tools::RealtimePublisher<experimental_controllers::JointPositionVelocityStampedArray>(node_handle_,
			std::string("joint_position_desired"), 1);
	joint_actual_array_publisher_ = new realtime_tools::RealtimePublisher<experimental_controllers::JointPositionVelocityStampedArray>(node_handle_, std::string(
			"joint_position_actual"), 1);

	std::vector<std::string> joint_names;
	std::vector<double> zeros;
	for (int i = 0; i < num_joints_; i++)
	{
		joint_names.push_back(mechanism_chain_.getJoint(i)->joint_->name);
		zeros.push_back(0.0);
	}
	for (int i = 0; i < publisher_buffer_size_; i++)
	{
		joint_desired_array_.array[i].header.seq = i;
		joint_actual_array_.array[i].header.seq = i;

		joint_desired_array_.array[i].names = joint_names;
		joint_actual_array_.array[i].names = joint_names;

		joint_desired_array_.array[i].velocities = zeros;
		joint_actual_array_.array[i].velocities = zeros;

		joint_desired_array_.array[i].positions = zeros;
		joint_actual_array_.array[i].positions = zeros;
	}

	joint_desired_array_publisher_->lock();
	joint_desired_array_publisher_->msg_ = joint_desired_array_;
	joint_desired_array_publisher_->unlock();

	joint_actual_array_publisher_->lock();
	joint_actual_array_publisher_->msg_ = joint_actual_array_;
	joint_actual_array_publisher_->unlock();

	pose_twist_desired_array_.array.resize(publisher_buffer_size_);
	pose_twist_actual_array_.array.resize(publisher_buffer_size_);
	// 	pose_twist_desired_array_publisher_.reset(new realtime_tools::RealtimePublisher<experimental_controllers::PoseTwistStampedArray>(node_handle_, std::string(
	// 			"pose_twist_desired"), 1));
	// 	pose_twist_actual_array_publisher_.reset(new realtime_tools::RealtimePublisher<experimental_controllers::PoseTwistStampedArray>(node_handle_, std::string(
	// 			"pose_twist_actual"), 1));
	pose_twist_desired_array_publisher_ = new realtime_tools::RealtimePublisher<experimental_controllers::PoseTwistStampedArray>(node_handle_, std::string(
			"pose_twist_desired"), 1);
	pose_twist_actual_array_publisher_ = new realtime_tools::RealtimePublisher<experimental_controllers::PoseTwistStampedArray>(node_handle_, std::string(
			"pose_twist_actual"), 1);

	geometry_msgs::Point zero_point;
	zero_point.x = 0.0;
	zero_point.y = 0.0;
	zero_point.z = 0.0;
	geometry_msgs::Quaternion zero_quat;
	zero_quat.x = 0.0;
	zero_quat.y = 0.0;
	zero_quat.z = 0.0;
	zero_quat.w = 1.0;
	geometry_msgs::Pose zero_pose;
	zero_pose.position = zero_point;
	zero_pose.orientation = zero_quat;

	geometry_msgs::Vector3 zero_vec;
	zero_vec.x = 0.0;
	zero_vec.y = 0.0;
	zero_vec.z = 0.0;
	geometry_msgs::Twist zero_twist;
	zero_twist.angular = zero_vec;
	zero_twist.linear = zero_vec;

	for (int i = 0; i < publisher_buffer_size_; i++)
	{
		pose_twist_desired_array_.array[i].header.seq = i;
		pose_twist_actual_array_.array[i].header.seq = i;

		pose_twist_desired_array_.array[i].pose = zero_pose;
		pose_twist_actual_array_.array[i].twist = zero_twist;
	}

	nullspace_term_array_.array.resize(publisher_buffer_size_);
	// 	nullspace_term_array_publisher_.reset(new realtime_tools::RealtimePublisher<experimental_controllers::NullspaceTermStampedArray>(node_handle_, std::string(
	// 			"nullspace_term"), 1));
	nullspace_term_array_publisher_ = new realtime_tools::RealtimePublisher<experimental_controllers::NullspaceTermStampedArray>(node_handle_, std::string(
			"nullspace_term"), 1);

	// check if joints are calibrated
	if (!mechanism_chain_.allCalibrated())
	{
		ROS_ERROR("CartesianTwistControllerIkWithNullspaceOptimization::starting>> joints are not calibrated.");
		return false;
	}

	return true;
}

void CartesianTwistControllerIkWithNullspaceOptimization::starting()
{

	// reset cartesian space pid controllers
	for (unsigned int i = 0; i < 6; i++)
	{
		cartesian_fb_pid_controllers_[i].reset();
	}

	for (int i = 0; i < num_joints_; i++)
	{
		nullspace_fb_pid_controllers_[i].reset();
	}

	// start joint velocity controller
	for (int i = 0; i < num_joints_; i++)
	{
		// joint_effort_controllers_[i]->starting();
		joint_velocity_controllers_[i]->starting();
	}

	// set desired twist to 0
	// TODO: check whether this is really necessary since it will be set later...
	kdl_twist_desired_ = KDL::Twist::Zero();

	// set measured twist to 0
	kdl_twist_measured_ = KDL::Twist::Zero();

	// get the joint positions and velocities
	mechanism_chain_.getPositions(kdl_current_joint_positions_);

	for (int i = 0; i < num_joints_; i++)
	{
		joint_positions_filter_values_[i] = kdl_current_joint_positions_(i);
	}
	joint_positions_filter_.update(joint_positions_filter_values_, joint_positions_filter_values_);
	for (int i = 0; i < num_joints_; i++)
	{
		kdl_current_joint_positions_(i) = joint_positions_filter_values_[i];
	}

	// normalize the joint angles for the continuous angles
	// kdl_current_joint_positions_(4) = angles::normalize_angle(kdl_current_joint_positions_(4));
	// kdl_current_joint_positions_(6) = angles::normalize_angle(kdl_current_joint_positions_(6));

	// set cartesian pose to current
	jnt_to_pose_solver_->JntToCart(kdl_current_joint_positions_, kdl_pose_desired_);

	kdl_pose_fixed_ = kdl_pose_desired_;

	last_time_ = robot_state_->getTime();
}

void CartesianTwistControllerIkWithNullspaceOptimization::update()
{

	// get time
	ros::Time time = robot_state_->getTime();
	ros::Duration dt = time - last_time_;
	last_time_ = time;

	// get the joint positions and filter them
	mechanism_chain_.getPositions(kdl_current_joint_positions_);

	for (int i = 0; i < num_joints_; i++)
	{
		joint_positions_filter_values_[i] = kdl_current_joint_positions_(i);
	}
	joint_positions_filter_.update(joint_positions_filter_values_, joint_positions_filter_values_);
	for (int i = 0; i < num_joints_; i++)
	{
		kdl_current_joint_positions_(i) = joint_positions_filter_values_[i];
	}

	// normalize the joint angles for the continuous angles
	// kdl_current_joint_positions_(4) = angles::normalize_angle(kdl_current_joint_positions_(4));
	// kdl_current_joint_positions_(6) = angles::normalize_angle(kdl_current_joint_positions_(6));

	// get the chain jacobian
	jnt_to_jac_solver_->JntToJac(kdl_current_joint_positions_, kdl_chain_jacobian_);

	// convert to (plain) eigen for easier math
	eigen_chain_jacobian_ = kdl_chain_jacobian_.data;

	// compute the pseudo inverse
	eigen_jac_times_jac_transpose_ = eigen_chain_jacobian_ * eigen_chain_jacobian_.transpose();
	eigen_jac_times_jac_transpose_.computeInverse(&eigen_jjt_inverse_);
	eigen_jac_pseudo_inverse_ = eigen_chain_jacobian_.transpose() * eigen_jjt_inverse_;

	// compute the nullspace projector
	eigen_nullspace_projector_ = eigen_identity_ - (eigen_jac_pseudo_inverse_ * eigen_chain_jacobian_);

	// get cartesian pose
	jnt_to_pose_solver_->JntToCart(kdl_current_joint_positions_, kdl_pose_measured_);

	// compute twist error
	if (keep_endeffector_fixed_for_testing_)
	{
		kdl_pose_desired_ = kdl_pose_fixed_;
	}
	// compute twist
	kdl_twist_error_ = -diff(kdl_pose_measured_, kdl_pose_desired_);

	// filter twist error
	for (int i = 0; i < 6; i++)
	{
		pose_unfiltered_data_[i] = kdl_twist_error_(i);
	}
	pose_filter_.update(pose_unfiltered_data_, pose_filtered_data_);
	for (int i = 0; i < 6; i++)
	{
		kdl_twist_error_(i) = pose_filtered_data_[i];
	}

	// compute desired cartesian velocities
	for (int i = 0; i < 3; i++)
	{
		eigen_desired_cartesian_velocities_(i) = (kdl_twist_desired_(i) * ff_trans_) + cartesian_fb_pid_controllers_[i].updatePid(kdl_twist_error_(i), dt);
	}
	for (int i = 3; i < 6; i++)
	{
		eigen_desired_cartesian_velocities_(i) = (kdl_twist_desired_(i) * ff_rot_) + cartesian_fb_pid_controllers_[i].updatePid(kdl_twist_error_(i), dt);
	}

	// compute desired joint velocities
	eigen_desired_joint_velocities_ = eigen_jac_pseudo_inverse_ * eigen_desired_cartesian_velocities_;

	double error;
	for (int i = 0; i < num_joints_; i++)
	{
		if (i == CONTINOUS_FOREARM_ROLL_JOINT)
		{
			error = angles::shortest_angular_distance(kdl_current_joint_positions_(i), rest_posture_joint_configuration_(i));
		}
		else if (i == CONTINOUS_WRIST_ROLL_JOINT)
		{
			error = angles::shortest_angular_distance(kdl_current_joint_positions_(i), rest_posture_joint_configuration_(i));
		}
		else
		{
			error = rest_posture_joint_configuration_(i) - kdl_current_joint_positions_(i);
		}
		eigen_nullspace_error_(i) = nullspace_fb_pid_controllers_[i].updatePid(error, dt);
	}
	eigen_nullspace_term_ = eigen_nullspace_projector_ * eigen_nullspace_error_;
	eigen_desired_joint_velocities_ = eigen_desired_joint_velocities_ + eigen_nullspace_term_;

	// 	Eigen::Matrix<double, 6, 1> computed_cartesian_vel;
	// 	computed_cartesian_vel = eigen_chain_jacobian_ * eigen_desired_joint_velocities_;

	// 	Eigen::Matrix<double, 6, 1> diff_vel;
	// 	diff_vel = eigen_desired_cartesian_velocities_ - computed_cartesian_vel;

	// 	ROS_ERROR_STREAM("eigen_desired_cartesian_velocities_:" <<  eigen_desired_cartesian_velocities_);
	// 	ROS_ERROR_STREAM("computed_cartesian_vel:" <<  computed_cartesian_vel);
	// 	ROS_ERROR_STREAM("diff_vel:" <<  diff_vel);
	// 	ROS_ERROR_STREAM(" ");
	// 	ROS_ERROR_STREAM(" ");

	// 	ROS_ERROR_STREAM("des_joint_vel:" <<  eigen_desired_joint_velocities_);
	// 	ROS_ERROR_STREAM("diff:" << eigen_desired_joint_velocities_ - (eigen_chain_jacobian_ * eigen_desired_joint_velocities_));

	// print matrices	
	// 	std::cout << "nullspace projector:" << std::endl;
	// 	std::cout << eigen_nullspace_projector_ << std::endl;

	// 	std::cout << "jacobian:" << std::endl;
	// 	std::cout << eigen_chain_jacobian_ << std::endl;

	// 	std::cout << "jacobian_inverse:" << std::endl;
	// 	std::cout << eigen_jac_pseudo_inverse_ << std::endl;

	// 	ROS_ERROR_STREAM("nullspace projector:");
	// 	ROS_ERROR_STREAM(eigen_nullspace_projector_);

	// 	ROS_ERROR_STREAM("jacobian:");
	// 	ROS_ERROR_STREAM(eigen_chain_jacobian_);

	// 	ROS_ERROR_STREAM("jacobian_inverse:");
	// 	ROS_ERROR_STREAM(eigen_jac_pseudo_inverse_);

	// set joint velocity set points and update
	for (int i = 0; i < num_joints_; i++)
	{
		// joint_effort_controllers_[i]->command_ = eigen_desired_joint_velocities_(i);
		joint_velocity_controllers_[i]->command_ = eigen_desired_joint_velocities_(i);
	}
	for (int i = 0; i < num_joints_; i++)
	{
		// joint_effort_controllers_[i]->update();
		joint_velocity_controllers_[i]->update();
	}

	// get measured twist for debugging reasons...
	KDL::FrameVel framevel_measured;
	mechanism_chain_.getVelocities(kdl_current_joint_velocities_);
	jnt_to_twist_solver_->JntToCart(kdl_current_joint_velocities_, framevel_measured);
	kdl_twist_measured_ = framevel_measured.deriv();

	aggregateAndPublish();
}

void CartesianTwistControllerIkWithNullspaceOptimization::stopping()
{
	for (int i = 0; i < num_joints_; i++)
	{
		// joint_effort_controllers_[i]->stopping();
		joint_velocity_controllers_[i]->stopping();
	}

}

void CartesianTwistControllerIkWithNullspaceOptimization::aggregateAndPublish()
{

	if (desired_joint_buffer_counter_ < publisher_buffer_size_)
	{
		joint_desired_array_.array[desired_joint_buffer_counter_].header.seq = joint_desired_array_.array[desired_joint_buffer_counter_].header.seq
				+ publisher_buffer_size_;
		joint_desired_array_.array[desired_joint_buffer_counter_].header.stamp = ros::Time::now();
		for (int i = 0; i < num_joints_; i++)
		{
			joint_desired_array_.array[desired_joint_buffer_counter_].positions[i] = rest_posture_joint_configuration_(i);
			joint_desired_array_.array[desired_joint_buffer_counter_].velocities[i] = eigen_desired_joint_velocities_(i);
		}
		desired_joint_buffer_counter_++;
	}
	if (desired_joint_buffer_counter_ == publisher_buffer_size_)
	{
		if (joint_desired_array_publisher_->trylock())
		{
			joint_desired_array_publisher_->msg_ = joint_desired_array_;
			joint_desired_array_publisher_->unlockAndPublish();
			desired_joint_buffer_counter_ = 0;
		}
	}

	if (actual_joint_buffer_counter_ < publisher_buffer_size_)
	{
		joint_actual_array_.array[actual_joint_buffer_counter_].header.seq = joint_actual_array_.array[actual_joint_buffer_counter_].header.seq
				+ publisher_buffer_size_;
		joint_actual_array_.array[actual_joint_buffer_counter_].header.stamp = ros::Time::now();
		for (int i = 0; i < num_joints_; i++)
		{
			joint_actual_array_.array[actual_joint_buffer_counter_].positions[i] = kdl_current_joint_positions_(i);
			joint_actual_array_.array[actual_joint_buffer_counter_].velocities[i] = kdl_current_joint_velocities_.qdot(i);
		}
		actual_joint_buffer_counter_++;
	}
	if (actual_joint_buffer_counter_ == publisher_buffer_size_)
	{
		if (joint_actual_array_publisher_->trylock())
		{
			joint_actual_array_publisher_->msg_ = joint_actual_array_;
			joint_actual_array_publisher_->unlockAndPublish();
			actual_joint_buffer_counter_ = 0;
		}
	}

	double qx, qy, qz, qw;
	if (desired_pose_twist_buffer_counter_ < publisher_buffer_size_)
	{
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].header.seq
				= pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].header.seq + publisher_buffer_size_;
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].header.stamp = ros::Time::now();

		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].pose.position.x = kdl_pose_desired_.p.x();
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].pose.position.y = kdl_pose_desired_.p.y();
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].pose.position.z = kdl_pose_desired_.p.z();
		kdl_pose_desired_.M.GetQuaternion(qx, qy, qz, qw);
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].pose.orientation.x = qx;
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].pose.orientation.y = qy;
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].pose.orientation.z = qz;
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].pose.orientation.w = qw;

		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].twist.linear.x = kdl_twist_desired_.vel.x();
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].twist.linear.y = kdl_twist_desired_.vel.y();
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].twist.linear.z = kdl_twist_desired_.vel.z();
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].twist.angular.x = kdl_twist_desired_.rot.x();
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].twist.angular.y = kdl_twist_desired_.rot.y();
		pose_twist_desired_array_.array[desired_pose_twist_buffer_counter_].twist.angular.z = kdl_twist_desired_.rot.z();

		desired_pose_twist_buffer_counter_++;
	}
	if (desired_pose_twist_buffer_counter_ == publisher_buffer_size_)
	{
		if (pose_twist_desired_array_publisher_->trylock())
		{
			pose_twist_desired_array_publisher_->msg_ = pose_twist_desired_array_;
			pose_twist_desired_array_publisher_->unlockAndPublish();
			desired_pose_twist_buffer_counter_ = 0;
		}
	}

	if (actual_pose_twist_buffer_counter_ < publisher_buffer_size_)
	{
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].header.seq = pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].header.seq
				+ publisher_buffer_size_;
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].header.stamp = ros::Time::now();

		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].pose.position.x = kdl_pose_measured_.p.x();
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].pose.position.y = kdl_pose_measured_.p.y();
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].pose.position.z = kdl_pose_measured_.p.z();
		kdl_pose_measured_.M.GetQuaternion(qx, qy, qz, qw);
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].pose.orientation.x = qx;
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].pose.orientation.y = qy;
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].pose.orientation.z = qz;
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].pose.orientation.w = qw;

// 		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.linear.x = kdl_twist_measured_.vel.x();
// 		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.linear.y = kdl_twist_measured_.vel.y();
// 		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.linear.z = kdl_twist_measured_.vel.z();
// 		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.angular.x = kdl_twist_measured_.rot.x();
// 		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.angular.y = kdl_twist_measured_.rot.y();
// 		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.angular.z = kdl_twist_measured_.rot.z();

		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.linear.x = eigen_desired_cartesian_velocities_(0);
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.linear.y = eigen_desired_cartesian_velocities_(1);
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.linear.z = eigen_desired_cartesian_velocities_(2);
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.angular.x = eigen_desired_cartesian_velocities_(3);
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.angular.y = eigen_desired_cartesian_velocities_(4);
		pose_twist_actual_array_.array[actual_pose_twist_buffer_counter_].twist.angular.z = eigen_desired_cartesian_velocities_(5);
		
		actual_pose_twist_buffer_counter_++;
	}
	if (actual_pose_twist_buffer_counter_ == publisher_buffer_size_)
	{
		if (pose_twist_actual_array_publisher_->trylock())
		{
			pose_twist_actual_array_publisher_->msg_ = pose_twist_actual_array_;
			pose_twist_actual_array_publisher_->unlockAndPublish();
			actual_pose_twist_buffer_counter_ = 0;
		}
	}

	if (nullspace_term_buffer_counter_ < publisher_buffer_size_)
	{
 		//nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_0 = eigen_nullspace_term_(0);
 		//nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_1 = eigen_nullspace_term_(1);
 		//nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_2 = eigen_nullspace_term_(2);
 		//nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_3 = eigen_nullspace_term_(3);
 		//nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_4 = eigen_nullspace_term_(4);
 		//nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_5 = eigen_nullspace_term_(5);
		//nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_6 = eigen_nullspace_term_(6);

 		nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_0 = eigen_nullspace_error_(0);
 		nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_1 = eigen_nullspace_error_(1);
 		nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_2 = eigen_nullspace_error_(2);
 		nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_3 = eigen_nullspace_error_(3);
 		nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_4 = eigen_nullspace_error_(4);
 		nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_5 = eigen_nullspace_error_(5);
		nullspace_term_array_.array[nullspace_term_buffer_counter_].nullspace_term_6 = eigen_nullspace_error_(6);

		nullspace_term_buffer_counter_++;
	}
	if (nullspace_term_buffer_counter_ == publisher_buffer_size_)
	{
		if (nullspace_term_array_publisher_->trylock())
		{
			nullspace_term_array_publisher_->msg_ = nullspace_term_array_;
			nullspace_term_array_publisher_->unlockAndPublish();
			nullspace_term_buffer_counter_ = 0;
		}
	}

}

void CartesianTwistControllerIkWithNullspaceOptimization::computeAngularVelocityError(const double* quat1, const double* quat2, double* angular_velocity_error)
{
	angular_velocity_error[0] = quat1[0] * quat2[1] - quat2[0] * quat1[1] - (quat1[2] * quat2[3] - quat1[3] * quat2[2]);
	angular_velocity_error[1] = quat1[0] * quat2[2] - quat2[0] * quat1[2] - (quat1[3] * quat2[1] - quat1[1] * quat2[3]);
	angular_velocity_error[2] = quat1[0] * quat2[3] - quat2[0] * quat1[3] - (quat1[1] * quat2[2] - quat1[2] * quat2[1]);
}

void CartesianTwistControllerIkWithNullspaceOptimization::getQuaternionFromRPY(const double roll, const double pitch, const double yaw, double &qx, double &qy,
		double &qz, double &qw)
{
	qw = cos(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
	qx = sin(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) - cos(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
	qy = cos(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0);
	qz = cos(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0) - sin(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0);
}

} // namespace
