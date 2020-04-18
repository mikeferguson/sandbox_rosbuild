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

// Author: Stuart Glaser

#ifndef CARTESIAN_HYBRID_CONTROLLER_H
#define CARTESIAN_HYBRID_CONTROLLER_H

#include "experimental_controllers/cartesian_hybrid_controller.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include "tf/tfMessage.h"
#include "tf_conversions/tf_kdl.h"
#include "realtime_tools/realtime_publisher.h"
#include "control_toolbox/pid.h"
#include "visualization_msgs/Marker.h"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

#include "std_msgs/Float64MultiArray.h"

PLUGINLIB_REGISTER_CLASS(CartesianHybridController, controller::CartesianHybridController, pr2_controller_interface::Controller)
PLUGINLIB_REGISTER_CLASS(CartesianHybridControllerNode, controller::CartesianHybridControllerNode, pr2_controller_interface::Controller)

namespace controller {

void TransformKDLToMsg(const KDL::Frame &k, geometry_msgs::Pose &m)
{
  tf::Transform tf;
  tf::TransformKDLToTF(k, tf);
  tf::poseTFToMsg(tf, m);
}

void TwistKDLToMsg(const KDL::Twist &k, geometry_msgs::Twist &m)
{
  m.linear.x = k.vel.x();
  m.linear.y = k.vel.y();
  m.linear.z = k.vel.z();
  m.angular.x = k.rot.x();
  m.angular.y = k.rot.y();
  m.angular.z = k.rot.z();
}

void WrenchKDLToMsg(const KDL::Wrench &k, geometry_msgs::Wrench &m)
{
  m.force.x = k.force.x();
  m.force.y = k.force.y();
  m.force.z = k.force.z();
  m.torque.x = k.torque.x();
  m.torque.y = k.torque.y();
  m.torque.z = k.torque.z();
}

CartesianHybridController::CartesianHybridController()
  : robot_(NULL), last_time_(0), use_filter_(false), twist_filter_("double")
{
}

bool CartesianHybridController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  node_ = n;
  assert(robot);
  robot_ = robot;

  // Chain

  std::string root_link, tip_link;
  if (!node_.getParam("root_link", root_link)) {
    ROS_ERROR("No root link specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tip_link", tip_link)) {
    ROS_ERROR("No tip link specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!chain_.init(robot, root_link, tip_link))
    return false;
  chain_.toKDL(kdl_chain_);

  // Pids

  control_toolbox::Pid temp_pid;

  if (!temp_pid.init(ros::NodeHandle(node_, "pose/fb_trans")))
    return false;
  for (size_t i = 0; i < 3; ++i)
    pose_pids_[i] = temp_pid;

  if (!temp_pid.init(ros::NodeHandle(node_, "pose/fb_rot")))
    return false;
  for (size_t i = 0; i < 3; ++i)
    pose_pids_[i+3] = temp_pid;

  if (!temp_pid.init(ros::NodeHandle(node_, "twist/fb_trans")))
    return false;
  for (size_t i = 0; i < 3; ++i)
    twist_pids_[i] = temp_pid;

  if (!temp_pid.init(ros::NodeHandle(node_, "twist/fb_rot")))
    return false;
  for (size_t i = 0; i < 3; ++i)
    twist_pids_[i+3] = temp_pid;

  // Pid gain setters

  for (int i = 0; i < 3; ++i)
    pose_pid_tuner_.add(pose_pids_ + i);
  pose_pid_tuner_.advertise(ros::NodeHandle(node_, "pose"));
  for (int i = 3; i < 6; ++i)
    pose_rot_pid_tuner_.add(pose_pids_ + i);
  pose_rot_pid_tuner_.advertise(ros::NodeHandle(node_, "pose_rot"));
  for (int i = 0; i < 3; ++i)
    twist_pid_tuner_.add(twist_pids_ + i);
  twist_pid_tuner_.advertise(ros::NodeHandle(node_, "twist"));
  for (int i = 3; i < 6; ++i)
    twist_rot_pid_tuner_.add(twist_pids_ + i);
  twist_rot_pid_tuner_.advertise(ros::NodeHandle(node_, "twist_rot"));


  // Filter

  if (node_.hasParam("twist_filter"))
  {
    use_filter_ = true;

    // \TODO remove when ticket https://prdev.willowgarage.com/trac/personalrobots/ticket/2575 is resolved
    std::string filter_xml;
    node_.getParam("twist_filter", filter_xml);
    int offset = 0;
    XmlRpc::XmlRpcValue filter_xmlrpc(filter_xml, &offset);
    if (!twist_filter_.configure(6, filter_xmlrpc))
      return false;
    /* Replace with below
    if (!twist_filter_.configure(6, "twist_filter", node_))
      return false;
      end Replace */
    ROS_INFO("Successfully configured twist_filter (namespace: %s)",
             node_.getNamespace().c_str());
  }
  else
    use_filter_ = false;

  // Initial mode
  node_.param("initial_mode", initial_mode_, manipulation_msgs::TaskFrameFormalism::FORCE);

  // Saturated velocity
  node_.param("saturated_velocity", saturated_velocity_, -1.0);
  node_.param("saturated_rot_velocity", saturated_rot_velocity_, -1.0);

  // Tool frame

  if (node_.hasParam("tool_frame"))
  {
    if (!node_.getParam("tool_frame/translation/x", tool_frame_offset_.p[0]) ||
        !node_.getParam("tool_frame/translation/y", tool_frame_offset_.p[1]) ||
        !node_.getParam("tool_frame/translation/z", tool_frame_offset_.p[2]))
    {
      ROS_ERROR("Tool frame was missing elements of the translation");
      return false;
    }
    tf::Quaternion q;
    if (!node_.getParam("tool_frame/rotation/x", q[0]) ||
        !node_.getParam("tool_frame/rotation/y", q[1]) ||
        !node_.getParam("tool_frame/rotation/z", q[2]) ||
        !node_.getParam("tool_frame/rotation/w", q[3]))
    {
      ROS_ERROR("Tool frame was missing elements of the rotation");
      return false;
    }
    tool_frame_offset_.M = KDL::Rotation::Quaternion(q[0], q[1], q[2], q[3]);
  }
  else
  {
    ROS_DEBUG("No tool frame specified");
    tool_frame_offset_ = KDL::Frame::Identity();
  }

  // Default commands

  task_frame_offset_ = KDL::Frame::Identity();

  // allocate vector in non-realtime
  measured_torque_.resize(kdl_chain_.getNrOfJoints());
  desired_torque_.resize(kdl_chain_.getNrOfJoints());
  max_jnt_eff_.resize(kdl_chain_.getNrOfJoints());

  // set default max jnt efforts
  for (unsigned int i=0; i<kdl_chain_.getNrOfJoints(); i++)
    max_jnt_eff_[i] = 100;

  // @TODO: remove this ugly setting of joint effort limits
  if (kdl_chain_.getNrOfJoints() >= 7){
    max_jnt_eff_[0] = 12;
    max_jnt_eff_[1] = 12;
    max_jnt_eff_[2] = 7;
    max_jnt_eff_[3] = 7;
    max_jnt_eff_[4] = 12;
    max_jnt_eff_[5] = 10;
    max_jnt_eff_[6] = 10;
  }

  return true;
}


void CartesianHybridController::update()
{
  if (!chain_.allCalibrated())
    return;
  ros::Time time = robot_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  // Measures the current pose and twist

  // Finds the pose/twist of the ee frame w.r.t. the chain root
  KDL::JntArrayVel jnt_vel(kdl_chain_.getNrOfJoints());
  chain_.getVelocities(jnt_vel);
  KDL::FrameVel ee_in_root;
  KDL::ChainFkSolverVel_recursive fkvel_solver(kdl_chain_);
  fkvel_solver.JntToCart(jnt_vel, ee_in_root);

  // The pose/twist of the tool frame w.r.t. the task frame
  KDL::FrameVel tool = task_frame_offset_.Inverse() * ee_in_root * tool_frame_offset_;
  pose_meas_ = tool.GetFrame();
  twist_meas_ = tool.GetTwist();

  // Computes the desired wrench from the command

  // Computes the filtered twist
  if (use_filter_)
  {
    std::vector<double> tmp_twist(6);
    for (size_t i = 0; i < 6; ++i)
      tmp_twist[i] = twist_meas_[i];
    twist_filter_.update(tmp_twist, tmp_twist);
    for (size_t i = 0; i < 6; ++i)
      twist_meas_filtered_[i] = tmp_twist[i];
  }
  else
  {
    twist_meas_filtered_ = twist_meas_;
  }

  // Computes the desired pose
  for (int i = 0; i < 6; ++i)
  {
    if (mode_[i] == manipulation_msgs::TaskFrameFormalism::POSITION)
      pose_desi_[i] = setpoint_[i];
    else// if (mode_[i] == manipulation_msgs::TaskFrameFormalism::VELOCITY)
    {
      pose_desi_[i] += setpoint_[i] * dt.toSec();
    }
    /*
    else
      pose_desi_[i] = 0.0;
    */
  }

  // Computes the pose error
  pose_error_.vel = tool.p.p - pose_desi_.vel;
  pose_error_.rot =
    diff(KDL::Rotation::RPY(
           mode_[3] == manipulation_msgs::TaskFrameFormalism::POSITION ? setpoint_[3] : 0.0,
           mode_[4] == manipulation_msgs::TaskFrameFormalism::POSITION ? setpoint_[4] : 0.0,
           mode_[5] == manipulation_msgs::TaskFrameFormalism::POSITION ? setpoint_[5] : 0.0),
         tool.M.R);

  // Computes the desired twist
  for (int i = 0; i < 6; ++i)
  {
    switch (mode_[i])
    {
    case manipulation_msgs::TaskFrameFormalism::POSITION:
      twist_desi_[i] = pose_pids_[i].updatePid(pose_error_[i], twist_meas_filtered_[i], dt);
      break;
    case manipulation_msgs::TaskFrameFormalism::VELOCITY:
      //twist_desi_[i] = setpoint_[i];
      twist_desi_[i] = pose_pids_[i].updatePid(pose_error_[i], twist_meas_filtered_[i] - setpoint_[i], dt);
      break;
    }
  }

  // Limits the velocity
  if (saturated_velocity_ >= 0.0)
  {
    if (twist_desi_.vel.Norm() > saturated_velocity_)
    {
      twist_desi_.vel = saturated_velocity_ * twist_desi_.vel / twist_desi_.vel.Norm();
    }
  }
  if (saturated_rot_velocity_ >= 0.0)
  {
    if (twist_desi_.rot.Norm() > saturated_rot_velocity_)
    {
      twist_desi_.rot = saturated_rot_velocity_ * twist_desi_.rot / twist_desi_.rot.Norm();
    }
  }

  for (int i = 0; i < 6; ++i)
  {
    twist_error_[i] = twist_meas_filtered_[i] - twist_desi_[i];
  }

  // Computes the desired wrench
  for (int i = 0; i < 6; ++i)
  {
    switch (mode_[i])
    {
    case manipulation_msgs::TaskFrameFormalism::POSITION:
      /*
      if (i >= 3) {
        wrench_desi_[i] = twist_desi_[i];
        break;
      }
      */
      wrench_desi_[i] = twist_desi_[i];
      break;
    case manipulation_msgs::TaskFrameFormalism::VELOCITY:
      //wrench_desi_[i] = twist_pids_[i].updatePid(twist_error_[i], dt);
      wrench_desi_[i] = twist_desi_[i];
      break;
    case manipulation_msgs::TaskFrameFormalism::FORCE:
      wrench_desi_[i] = setpoint_[i];
      break;
    default:
      abort();
    }
  }

  // Finds the Jacobian with reference frame root, and reference point tool
  KDL::ChainJntToJacSolver jac_solver(kdl_chain_);
  KDL::Jacobian ee_jacobian(kdl_chain_.getNrOfJoints());
  KDL::Jacobian jacobian(kdl_chain_.getNrOfJoints());
  // get jacobian with reference frame root, and reference point tip
  jac_solver.JntToJac(jnt_vel.q, ee_jacobian);
  // change reference point of jacobian from ee to tool
  KDL::changeRefPoint(ee_jacobian, ee_in_root.value().M * tool_frame_offset_.p, jacobian);

  // scale the force component in wrench_desi_ to prevent it from saturating the joint efforts
  KDL::Wrench push_force(wrench_desi_.force, KDL::Vector::Zero());
  push_force = task_frame_offset_.M * push_force;
  KDL::JntArray jnt_eff_push(kdl_chain_.getNrOfJoints());
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i){
    jnt_eff_push(i) = 0;
    for (size_t j = 0; j < 6; ++j)
      jnt_eff_push(i) += jacobian(j,i) * push_force(j);
  }
  double max_scale = 0;
  for (unsigned int i=0; i<kdl_chain_.getNrOfJoints(); i++){
    double scale = fabs(jnt_eff_push(i) / max_jnt_eff_[i]);
    if (scale > max_scale) max_scale = scale;
  }
  if (max_scale > 1.0)
    wrench_desi_.force = wrench_desi_.force / max_scale;

  // transform the reference frame from the task frame to the root frame
  KDL::Wrench wrench_in_root;
  wrench_in_root = task_frame_offset_.M * wrench_desi_;

  // jnt_eff = jacobian * wrench
  KDL::JntArray jnt_eff(kdl_chain_.getNrOfJoints());
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    jnt_eff(i) = 0;
    for (size_t j = 0; j < 6; ++j)
      jnt_eff(i) += jacobian(j,i) * wrench_in_root(j);
  }
  chain_.addEfforts(jnt_eff);

  // copy desired/measured joint torques in vector
  for (unsigned int i=0; i<kdl_chain_.getNrOfJoints(); i++){
    chain_.getEfforts(measured_torque_);
    desired_torque_[i] = jnt_eff(i);
  }

}

void CartesianHybridController::starting()
{
  task_frame_offset_ = KDL::Frame::Identity();
  //tool_frame_offset_ = KDL::Frame::Identity();

  for (int i = 0; i < 6; ++i) {
    pose_pids_[i].reset();
    twist_pids_[i].reset();
  }

  switch(initial_mode_)
  {
  case manipulation_msgs::TaskFrameFormalism::POSITION: {
    // Finds the starting pose/twist
    KDL::JntArrayVel jnt_vel(kdl_chain_.getNrOfJoints());
    chain_.getVelocities(jnt_vel);
    KDL::FrameVel frame;
    KDL::ChainFkSolverVel_recursive fkvel_solver(kdl_chain_);
    fkvel_solver.JntToCart(jnt_vel, frame);
    frame = frame * tool_frame_offset_;

    for (size_t i = 0; i < 6; ++i) {
      mode_[i] = initial_mode_;
    }
    for (size_t i = 0; i < 3; ++i) {
      setpoint_[i] = frame.p.p[i];
    }
    frame.M.R.GetRPY(setpoint_[3], setpoint_[4], setpoint_[5]);
    break;
  }
  case manipulation_msgs::TaskFrameFormalism::VELOCITY:
    for (size_t i = 0; i < 6; ++i) {
      mode_[i] = initial_mode_;
      setpoint_[i] = 0.0;
    }
    break;
  case manipulation_msgs::TaskFrameFormalism::FORCE:
    for (size_t i = 0; i < 6; ++i) {
      mode_[i] = initial_mode_;
      setpoint_[i] = 0.0;
    }
    break;
  default:
    ROS_FATAL("initial_mode_ is %d", initial_mode_);
    return false;
  }

  return true;
}

CartesianHybridControllerNode::CartesianHybridControllerNode()
//: TF(*ros::Node::instance(), false, ros::Duration(10.0)), loop_count_(0)
: loop_count_(0)
{
}

CartesianHybridControllerNode::~CartesianHybridControllerNode()
{
  //node->unsubscribe(name_ + "/command");
  //node->unadvertiseService(name_ + "/set_tool_frame");
}

bool CartesianHybridControllerNode::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  node_ = n;
  if (!c_.init(robot, n))
    return false;

  task_frame_name_ = c_.kdl_chain_.getSegment(0).getName();

  //node->subscribe(name_ + "/command", command_msg_, &CartesianHybridControllerNode::command, this, 5);
  command_notifier_.reset(new tf::MessageNotifier<manipulation_msgs::TaskFrameFormalism>(
                            TF,
                            boost::bind(&CartesianHybridControllerNode::command, this, _1),
                            node_.getNamespace() + "/command", c_.kdl_chain_.getSegment(0).getName(),
                            100));

  pub_state_.reset(new realtime_tools::RealtimePublisher<experimental_controllers::CartesianHybridState>
                   (node_, "state", 1));
  pub_tf_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>("/tf", 5));
  pub_tf_->msg_.transforms.resize(1);

  serve_set_tool_frame_ = node_.advertiseService("set_tool_frame",&CartesianHybridControllerNode::setToolFrame, this);

  // allocate vector in non-realtime
  pub_state_->msg_.measured_torque.resize(c_.kdl_chain_.getNrOfJoints());
  pub_state_->msg_.desired_torque.resize(c_.kdl_chain_.getNrOfJoints());

  return true;

}


void CartesianHybridControllerNode::update()
{

  KDL::Twist last_pose_desi = c_.pose_desi_;
  KDL::Twist last_twist_desi = c_.twist_desi_;
  KDL::Wrench last_wrench_desi = c_.wrench_desi_;

  c_.update();

  if (++loop_count_ % 10 == 0)
  {
    if (pub_state_->trylock())
    {
      pub_state_->msg_.header.frame_id = task_frame_name_;

      TwistKDLToMsg(c_.pose_error_, pub_state_->msg_.pose_error);
      TwistKDLToMsg(c_.twist_error_, pub_state_->msg_.twist_error);

      TwistKDLToMsg(c_.pose_desi_, pub_state_->msg_.last_pose_desi);
      TwistKDLToMsg(c_.twist_meas_, pub_state_->msg_.last_twist_meas);
      TwistKDLToMsg(c_.twist_meas_filtered_, pub_state_->msg_.last_twist_meas_filtered);
      TwistKDLToMsg(c_.twist_desi_, pub_state_->msg_.last_twist_desi);
      WrenchKDLToMsg(c_.wrench_desi_, pub_state_->msg_.last_wrench_desi);

      KDL::Twist last_pose_meas;
      last_pose_meas.vel = c_.pose_meas_.p;
      c_.pose_meas_.M.GetRPY(last_pose_meas.rot[0],
                             last_pose_meas.rot[1],
                             last_pose_meas.rot[2]);
      TwistKDLToMsg(last_pose_meas, pub_state_->msg_.last_pose_meas);

      pub_state_->msg_.desired_torque = c_.desired_torque_;
      pub_state_->msg_.measured_torque = c_.measured_torque_;

      pub_state_->unlockAndPublish();
    }
    if (pub_tf_->trylock())
    {
      //pub_tf_->msg_.transforms[0].header.stamp.fromSec();
      pub_tf_->msg_.transforms[0].header.frame_id = c_.kdl_chain_.getSegment(c_.kdl_chain_.getNrOfSegments()-1).getName();
      pub_tf_->msg_.transforms[0].child_frame_id = name_ + "/tool_frame";
      tf::Transform t;
      tf::TransformKDLToTF(c_.tool_frame_offset_, t);
      tf::transformTFToMsg(t, pub_tf_->msg_.transforms[0].transform);
      pub_tf_->unlockAndPublish();
    }
  }
}

void CartesianHybridControllerNode::command(
  const tf::MessageNotifier<manipulation_msgs::TaskFrameFormalism>::MessagePtr& tff_msg)
{
  task_frame_name_ = tff_msg->header.frame_id;
  tf::Stamped<tf::Transform> task_frame;

  try {
    TF.lookupTransform(c_.kdl_chain_.getSegment(0).getName(), tff_msg->header.frame_id, tff_msg->header.stamp,
                       task_frame);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }
  tf::TransformTFToKDL(task_frame, c_.task_frame_offset_);

  int old_modes[6];
  for (int i =0 ; i < 6; i++){
    old_modes[i] = c_.mode_[i];
  }
  c_.mode_[0] = (int)tff_msg->mode.linear.x;
  c_.mode_[1] = (int)tff_msg->mode.linear.y;
  c_.mode_[2] = (int)tff_msg->mode.linear.z;
  c_.mode_[3] = (int)tff_msg->mode.angular.x;
  c_.mode_[4] = (int)tff_msg->mode.angular.y;
  c_.mode_[5] = (int)tff_msg->mode.angular.z;
  c_.setpoint_[0] = tff_msg->value.linear.x;
  c_.setpoint_[1] = tff_msg->value.linear.y;
  c_.setpoint_[2] = tff_msg->value.linear.z;
  c_.setpoint_[3] = tff_msg->value.angular.x;
  c_.setpoint_[4] = tff_msg->value.angular.y;
  c_.setpoint_[5] = tff_msg->value.angular.z;

  for(int i = 0; i < 6; i++){
    if(old_modes[i] != c_.mode_[i]){
      c_.pose_pids_[i].reset();
      c_.twist_pids_[i].reset();
    }
  }
}

bool CartesianHybridControllerNode::setToolFrame(
  experimental_controllers::SetPoseStamped::Request &req,
  experimental_controllers::SetPoseStamped::Response &resp)
{
  if (!TF.waitForTransform(c_.kdl_chain_.getSegment(c_.kdl_chain_.getNrOfSegments()-1).getName(), req.p.header.frame_id,
                       req.p.header.stamp, ros::Duration(3.0)))
  {
    ROS_ERROR("Cannot transform %s -> %s at %lf", c_.kdl_chain_.getSegment(c_.kdl_chain_.getNrOfSegments()-1).getName().c_str(),
              req.p.header.frame_id.c_str(), req.p.header.stamp.toSec());
    return false;
  }

  geometry_msgs::PoseStamped tool_in_tip_msg;
  tf::Transform tool_in_tip;
  TF.transformPose(c_.kdl_chain_.getSegment(c_.kdl_chain_.getNrOfSegments()-1).getName(), req.p, tool_in_tip_msg);
  tf::poseMsgToTF(tool_in_tip_msg.pose, tool_in_tip);
  tool_in_tip.setOrigin(tf::Vector3(0,0,0));
  tf::TransformTFToKDL(tool_in_tip, c_.tool_frame_offset_);
  double rpy[3]; c_.tool_frame_offset_.M.GetRPY(rpy[0], rpy[1], rpy[2]);
  ROS_INFO("(%.3lf, %.3lf, %.3lf)@(%.2lf, %.2lf, %.2lf)",
           c_.tool_frame_offset_.p[0], c_.tool_frame_offset_.p[1], c_.tool_frame_offset_.p[2],
           rpy[0], rpy[1], rpy[2]);
  return true;
}

}

#endif
