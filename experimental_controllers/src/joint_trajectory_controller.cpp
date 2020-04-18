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

// Original version: Sachin Chitta <sachinc@willowgarage.com>

#include "experimental_controllers/joint_trajectory_controller.h"
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "pluginlib/class_list_macros.h"


PLUGINLIB_REGISTER_CLASS(JointTrajectoryController, controller::JointTrajectoryController, pr2_controller_interface::Controller)

using namespace controller;
using namespace std;

JointTrajectoryController::JointTrajectoryController() : Controller()
{
  controller_state_publisher_ = NULL;
  num_joints_ = 0;
  trajectory_type_ = "linear";
  trajectory_wait_time_= 0.0;
  max_update_time_= 0.0;
  watch_dog_active_ = false;
  request_trajectory_id_ = 0;
  current_trajectory_id_ = -1;
  trajectory_wait_timeout_ = 10.0;
  diagnostics_publisher_ = NULL;

  num_trajectory_available_ = 0;
  next_free_index_ = 0;
  current_trajectory_index_ = 0;
  trajectory_preempted_ = false;
  joint_trajectory_ = NULL;
}


JointTrajectoryController::~JointTrajectoryController()
{
  for(unsigned int i=0; i<joint_pv_controllers_.size();++i)
    if (joint_pv_controllers_[i])
      delete joint_pv_controllers_[i];


  stopPublishers();
  unadvertiseServices();
  unsubscribeTopics();

  if (joint_trajectory_)
    delete joint_trajectory_;
}

bool JointTrajectoryController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
  // get xml from parameter server
  string xml_string;
  if (!n.getParam("xml", xml_string)){
    ROS_ERROR("Could not read xml from parameter server");
    return false;
  }

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  if (xml_doc.Error())
  {
    ROS_ERROR("Error when parsing XML: %d (%d,%d)  %s\n%s", xml_doc.ErrorId(),
              xml_doc.ErrorRow(), xml_doc.ErrorCol(), xml_doc.ErrorDesc(), xml_string.c_str());
    return false;
  }
  TiXmlElement *xml = xml_doc.FirstChildElement("controller");
  if (!xml){
    ROS_ERROR("could not parse xml: %s\n %s", xml_doc.ErrorDesc(),xml_string.c_str());
    return false;
  }

  return initXml(robot, xml);
}


bool JointTrajectoryController::initXml(pr2_mechanism_model::RobotState * robot, TiXmlElement * config)
{
  name_ = config->Attribute("name");
  prefix_ = name_ + "/";

  getParams();

  if(!loadXmlFile(robot,config))
    {
      ROS_ERROR("Could not load controller from XML file. Check joint names");
      return false;
    }

  joint_trajectory_vector_.resize(max_trajectory_queue_size_);
  joint_trajectory_id_.resize(max_trajectory_queue_size_);

  subscribeTopics();

  advertiseServices();

  initializePublishers();


  ROS_INFO("Loaded JointTrajectoryController: %s",name_.c_str());
  return true;
}

void JointTrajectoryController::starting()
{
  current_time_ = robot_state_->getTime();
  updateJointValues();
  last_time_ = current_time_;
  setTrajectoryCmdToCurrentValues();
}

void JointTrajectoryController::getParams()
{
  double scale;
  node_.param<double>(prefix_+"velocity_scaling_factor",scale,0.25);
  node_.param<double>(prefix_+"trajectory_wait_timeout",trajectory_wait_timeout_,10.0);
  node_.param<std::string>(prefix_+"listen_topic_name",listen_topic_name_,"command");
  node_.param<double>(prefix_+"at_rest_velocity_threshold_",at_rest_velocity_threshold_,1e-5);
  node_.param<double>(prefix_+"max_allowed_update_time_",max_allowed_update_time_,100.0);
  node_.param<double>(prefix_+"diagnostics_publish_delta_time",diagnostics_publish_delta_time_,1.0);
  node_.param<int>(prefix_+"max_trajectory_queue_size",max_trajectory_queue_size_,100);
  velocity_scaling_factor_ = std::min(1.0,std::max(0.0,scale));

}

void JointTrajectoryController::initializePublishers()
{
  if (controller_state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete controller_state_publisher_ ;
  ros::NodeHandle n(prefix_);
  controller_state_publisher_ = new realtime_tools::RealtimePublisher <experimental_controllers::ControllerState> (n, "controller_state", 1) ;

  if (diagnostics_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete diagnostics_publisher_ ;
  ros::NodeHandle n2;
  diagnostics_publisher_ = new realtime_tools::RealtimePublisher <diagnostic_msgs::DiagnosticArray> (n2, "/diagnostics", 2) ;

  last_diagnostics_publish_time_ = robot_state_->getTime();
  node_.param<double>(prefix_+"diagnostics_publish_delta_time",diagnostics_publish_delta_time_,1.0);

  controller_state_publisher_->msg_.name = name_;
  ROS_INFO("Initialized publishers.");
}

void JointTrajectoryController::stopPublishers()
{
  if (controller_state_publisher_)
    delete controller_state_publisher_;
  if (diagnostics_publisher_)
    delete diagnostics_publisher_;
}

void JointTrajectoryController::advertiseServices()
{
  trajectory_start_srv_ = node_.advertiseService(prefix_+"TrajectoryStart", &JointTrajectoryController::setJointTrajSrv, this);
  trajectory_query_srv_ = node_.advertiseService(prefix_+"TrajectoryQuery", &JointTrajectoryController::queryJointTrajSrv, this);
  trajectory_cancel_srv_= node_.advertiseService(prefix_+"TrajectoryCancel", &JointTrajectoryController::cancelJointTrajSrv, this);

  ROS_INFO("Service for setting trajectories : %sTrajectoryStart",prefix_.c_str());
  ROS_INFO("Service for querying trajectories : %sTrajectoryQuery",prefix_.c_str());
  ROS_INFO("Service for canceling trajectories : %sTrajectoryCancel",prefix_.c_str());
}

void JointTrajectoryController::unadvertiseServices()
{
}

void JointTrajectoryController::subscribeTopics()
{
  cmd_sub_ =   node_.subscribe(prefix_+listen_topic_name_, 1, &JointTrajectoryController::TrajectoryReceivedOnTopic, this);
  ROS_INFO("Listening to topic: %s%s",prefix_.c_str(),listen_topic_name_.c_str());
}

void JointTrajectoryController::unsubscribeTopics()
{
  //  node_.unsubscribe(prefix_ + listen_topic_name_);
}

bool JointTrajectoryController::loadXmlFile(pr2_mechanism_model::RobotState * robot, TiXmlElement * config)
{
  ROS_DEBUG("Loading joint trajectory controller from XML.");

  robot_ = robot->model_;
  robot_state_ = robot;
  current_time_ = robot_state_->getTime();
  TiXmlElement *elt = config->FirstChildElement("controller");
  while (elt)
  {
    if(static_cast<std::string>(elt->Attribute("type")) == std::string("JointPDController"))
    {
      JointPDController * jpc = new JointPDController();
      ROS_INFO("Joint PD Controller: %s , %s",(elt->Attribute("type")),(elt->Attribute("name")));

      joint_pv_controllers_.push_back(jpc);
      if(!jpc->initXml(robot, elt))
        return false;

      addJoint(jpc->getJointName());
    }
    else
    {
      ROS_ERROR("Unrecognized joint controller type: %s",(static_cast<std::string>(elt->Attribute("type"))).c_str());
    }
    elt = elt->NextSiblingElement("controller");
  }

  initTrajectory(config);

  joint_cmd_.resize(num_joints_);
  joint_cmd_dot_.resize(num_joints_);

  last_time_ = current_time_;
  last_traj_req_time_ = current_time_;

  goal_reached_threshold_.resize(num_joints_);
  max_allowable_joint_errors_.resize(num_joints_);
  for(int i=0; i< num_joints_;i++)
  {
    node_.param<double>(prefix_+ joint_name_[i] + "/goal_reached_threshold",goal_reached_threshold_[i],GOAL_REACHED_THRESHOLD);
    node_.param<double>(prefix_+ joint_name_[i] + "/joint_error_threshold",max_allowable_joint_errors_[i],MAX_ALLOWABLE_JOINT_ERROR_THRESHOLD);
  }

  ROS_INFO("Initialized joint trajectory controller");

  return true;
}


void JointTrajectoryController::initTrajectory(TiXmlElement * config)
{
  TiXmlElement *elt = config->FirstChildElement("trajectory");
  if(!elt)
    ROS_WARN("No trajectory information in xml file. ");
  else
  {
    trajectory_type_ = std::string(elt->Attribute("interpolation"));
    ROS_INFO("JointTrajectoryController:: interpolation type:: %s",trajectory_type_.c_str());
  }

  joint_trajectory_ = new trajectory::Trajectory((int) num_joints_);
  joint_trajectory_->setMaxRates(joint_velocity_limits_);
  joint_trajectory_->setInterpolationMethod(trajectory_type_);

  for(int i=0; i<num_joints_; i++)
  {
    if(joint_type_[i] == pr2_mechanism_model::JOINT_CONTINUOUS)
    {
      ROS_INFO("Setting joint %d to wrap",i);
      joint_trajectory_->setJointWraps(i);
    }
  }

  trajectory_start_time_ = current_time_;
  trajectory_end_time_ = current_time_;
  joint_trajectory_->autocalc_timing_ = true;

  current_joint_position_vector_.resize(2);
  current_joint_position_vector_[0].setDimension(num_joints_);
  current_joint_position_vector_[1].setDimension(num_joints_);

  trajectory_point_.setDimension(num_joints_);

}

void JointTrajectoryController::setTrajectoryCmdToCurrentValues()
{
  for(int i=0; i < 2; i++)
  {
    for(int j=0; j < num_joints_; j++)
    {
      current_joint_position_vector_[i].q_[j] = current_joint_position_[j];
    }
    current_joint_position_vector_[i].time_ = 0.0;
  }
//  ROS_DEBUG("Size of trajectory points vector : %d",current_joint_position_vector_.size());

  if(setTrajectoryCmd(current_joint_position_vector_))
  {
    current_trajectory_id_ = -1;
  }
}


void JointTrajectoryController::addJoint(const std::string &name)
{
  pr2_mechanism_model::JointState *js;
  js = robot_state_->getJointState(name);
  if(!js)
  {
    ROS_ERROR("Joint %s not found in class robot",name.c_str());
    return;
  }
  if(!js->joint_->limits)
  {
    ROS_ERROR("Joint %s has no limits specified",name.c_str());
    return;
  }

  joint_velocity_limits_.push_back(js->joint_->limits->velocity*velocity_scaling_factor_);
  joint_type_.push_back(js->joint_->type);
  joint_name_.push_back(name);

  current_joint_position_.push_back(0.0);
  current_joint_velocity_.push_back(0.0);

  joint_position_errors_.push_back(0.0);
  joint_velocity_errors_.push_back(0.0);
  num_joints_++;
}


bool JointTrajectoryController::setTrajectoryCmd(const std::vector<trajectory::Trajectory::TPoint>& joint_trajectory)
{
  if(joint_trajectory.size() < 1)
  {
    ROS_WARN("JointTrajectoryController:: No points in trajectory");
    return false;
  }
  goal_ = joint_trajectory.back();
  joint_trajectory_->setTrajectory(joint_trajectory);
  return true;
}

bool JointTrajectoryController::errorsWithinThreshold()
{
  for(int i=0; i < num_joints_; i++)
  {
    if(fabs(joint_position_errors_[i]) > max_allowable_joint_errors_[i])
    {
      return false;
    }
  }
  return true;
}

bool JointTrajectoryController::atRest()
{
  for(int i=0; i < num_joints_; i++)
  {
    if(fabs(current_joint_velocity_[i]) > at_rest_velocity_threshold_)
    {
      return false;
    }
  }
  return true;
}

void JointTrajectoryController::computeJointErrors()
{
  for(unsigned int i=0; i < (unsigned int) num_joints_; i++)
  {
    if(joint_type_[i] == pr2_mechanism_model::JOINT_CONTINUOUS)
    {
      joint_position_errors_[i] = angles::shortest_angular_distance(joint_cmd_[i], current_joint_position_[i]);
    }
    else if(joint_type_[i] == pr2_mechanism_model::JOINT_ROTARY)
    {
      joint_position_errors_[i] = angles::shortest_angular_distance(joint_cmd_[i], current_joint_position_[i]);
    }
    else
    {
      joint_position_errors_[i] = current_joint_position_[i] - joint_cmd_[i];
    }
    joint_velocity_errors_[i] = current_joint_velocity_[i] - joint_cmd_dot_[i];
  }
}

bool JointTrajectoryController::checkWatchDog(ros::Time current_time)
{
  if((current_time - last_traj_req_time_).toSec() < max_allowed_update_time_)
  {
    if(watch_dog_active_)
    {
      watch_dog_active_ = false;
    }
    return false;
  }
  return true;
}

void JointTrajectoryController::stopMotion()
{
  setTrajectoryCmdToCurrentValues();
}

void JointTrajectoryController::resetTrajectoryTimes()
{
  trajectory_start_time_ = current_time_;
  trajectory_wait_time_ = 0.0;
  trajectory_end_time_ = trajectory_start_time_ + ros::Duration(joint_trajectory_->getTotalTime());
//  ROS_INFO("Resetting trajectory time");
//  ROS_INFO("Trajectory time: %f",joint_trajectory_->getTotalTime());
}

void JointTrajectoryController::getJointCommands()
{
  double sample_time(0.0);
  sample_time = (current_time_ - trajectory_start_time_).toSec();
  joint_trajectory_->sample(trajectory_point_,sample_time);

  for(int i=0; i < num_joints_; ++i)
  {
    joint_cmd_[i] = trajectory_point_.q_[i];
    joint_cmd_dot_[i] = trajectory_point_.qdot_[i];
  }
}

bool JointTrajectoryController::trajectoryDone()
{

  if(current_trajectory_id_ == -1)
    return false;

  if(reachedGoalPosition(joint_cmd_))
  {
     updateTrajectoryQueue(current_trajectory_id_,JointTrajectoryController::DONE);
     return true;
  }

  if(current_time_ >= trajectory_end_time_)
  {
    trajectory_wait_time_ = (current_time_ - trajectory_end_time_).toSec();
    if(trajectory_wait_time_ >= trajectory_wait_timeout_)
    {
      trajectory_wait_time_ = 0.0;
      updateTrajectoryQueue(current_trajectory_id_,JointTrajectoryController::FAILED);
      return true;
    }
  }
  return false;
}

void JointTrajectoryController::update(void)
{
#ifdef PUBLISH_MAX_TIME
  double start_time = ros::Time::now().toSec();
#endif
  current_time_ = robot_state_->getTime();
  updateJointValues();
  int get_trajectory_index = 0;

/** Actual decision making done here
    (a) check if watch dog is activated, if it is preempt current trajectory and set command to current desired values,  set current_trajectory_finished to true
    (b) check if current trajectory was preempted, if it is, set status to CANCELED, set current_trajectory_finished to true
    (c) check if current trajectory is done, if it is, set its status to DONE or FAILED if timeout was exceeded, set current_trajectory_finished to true
    THEN
    (d) if current trajectory is finished, get a new trajectory from the queue, if no trajectory is available, set current desired poisitions to current joint positions
        effectively bringing the robot to rest.
**/

  if(checkWatchDog(current_time_))
  {
    if(!watch_dog_active_)
    {
      watch_dog_active_ = true;
      updateTrajectoryQueue(current_trajectory_id_,JointTrajectoryController::CANCELED);
      setTrajectoryCmdToCurrentValues();
      current_trajectory_id_ = -1;
    }
  }
  else if(trajectory_preempted_)
  {
    updateTrajectoryQueue(current_trajectory_id_,JointTrajectoryController::CANCELED);
    get_trajectory_index = current_trajectory_index_;
    trajectory_preempted_ = false;
    if(getTrajectoryFromQueue(get_trajectory_index))
    {
      current_trajectory_index_ = get_trajectory_index;
      resetTrajectoryTimes();
    }
    else
    {
      setTrajectoryCmdToCurrentValues();
      current_trajectory_id_ = -1;
    }
  }
  else if(trajectoryDone())
  {
    get_trajectory_index = (current_trajectory_index_ + 1)%max_trajectory_queue_size_;
    if(getTrajectoryFromQueue(get_trajectory_index))
    {
      current_trajectory_index_ = get_trajectory_index;
      resetTrajectoryTimes();
    }
    else
    {
      setTrajectoryCmdToCurrentValues();
      current_trajectory_id_ = -1;
    }
  }
  else if(current_trajectory_id_ == -1)
  {
    get_trajectory_index = (current_trajectory_index_)%max_trajectory_queue_size_;
    if(getTrajectoryFromQueue(get_trajectory_index))
    {
      current_trajectory_index_ = get_trajectory_index;
      resetTrajectoryTimes();
    }
  }

  getJointCommands();

  updateJointControllers();


#ifdef PUBLISH_MAX_TIME
  double end_time = ros::Time::now().toSec();
  max_update_time_ = std::max(max_update_time_,end_time-start_time);

  if (controller_state_publisher_->trylock())
  {
    controller_state_publisher_->msg_.update_time = end_time - start_time;
    controller_state_publisher_->msg_.max_update_time = max_update_time_;
    controller_state_publisher_->unlockAndPublish();
  }
#endif

  publishDiagnostics();

  last_time_ = current_time_;
}

bool JointTrajectoryController::reachedGoalPosition(std::vector<double> joint_cmd)
{
  bool return_val = true;
  double error(0.0);
  for(int i=0;i < num_joints_;++i)
  {
    if(joint_type_[i] == pr2_mechanism_model::JOINT_CONTINUOUS || joint_type_[i] == pr2_mechanism_model::JOINT_ROTARY)
    {
      error = fabs(angles::shortest_angular_distance(goal_.q_[i], current_joint_position_[i]));
    }
    else //prismatic
    {
      error = fabs(current_joint_position_[i] - goal_.q_[i]);
    }
    return_val = return_val && (error <= goal_reached_threshold_[i]);
  }
  return return_val;
}

void JointTrajectoryController::updateJointControllers(void)
{
  //compute errors
  computeJointErrors();

  // Set the commands for all the joints
  for(unsigned int i=0;i<joint_pv_controllers_.size();++i)
  {
    joint_pv_controllers_[i]->setCommand(joint_cmd_[i],joint_cmd_dot_[i]);
  }

  // Call update on all the controllers
  for(unsigned int i=0;i<joint_pv_controllers_.size();++i)
    joint_pv_controllers_[i]->update();
}

void JointTrajectoryController::updateJointValues()
{
  // Grab the current odometric position
  double q[3], qdot[3];

  for(unsigned int i=0; i < joint_pv_controllers_.size();++i){
    current_joint_position_[i] = joint_pv_controllers_[i]->joint_state_->position_;
    current_joint_velocity_[i] = joint_pv_controllers_[i]->joint_state_->velocity_;
  }

}

void JointTrajectoryController::updateTrajectoryQueue(int id, int finish_status)
{
  if(num_trajectory_available_ > 0 && id >= 0)
  {
    joint_trajectory_status_[id] = finish_status;
    joint_trajectory_time_[id] = (trajectory_end_time_ - trajectory_start_time_).toSec();
    if(finish_status != JointTrajectoryController::ACTIVE)
      num_trajectory_available_--;
  }
}

bool JointTrajectoryController::getTrajectoryFromQueue(int &index)
{
  if(num_trajectory_available_ < 1)
  {
//    ROS_INFO("No trajectories available");
    return false;
  }
  if(joint_trajectory_status_[joint_trajectory_id_[index]] == JointTrajectoryController::QUEUED)
  {
    if(trajectory_queue_.try_lock())
    {
      setTrajectoryCmdFromMsg(joint_trajectory_vector_[index],joint_trajectory_id_[index]);
      trajectory_queue_.unlock();
      return true;
    }
    return false;
  }
  else
  {
// do a linear search from current index + 1
    int iter = (index + 1)%max_trajectory_queue_size_;
    int num_iterations = 0;
    while(num_iterations < max_trajectory_queue_size_)
    {
      if(iter >= (int)joint_trajectory_status_.size())
      {
        iter = 0;
        continue;
      }
      if(joint_trajectory_status_[joint_trajectory_id_[iter]] == JointTrajectoryController::QUEUED)
      {
        if(trajectory_queue_.try_lock())
        {
          setTrajectoryCmdFromMsg(joint_trajectory_vector_[iter],joint_trajectory_id_[iter]);
          index = iter;
	  trajectory_queue_.unlock();
          return true;
        }
      }
      num_iterations++;
      iter = (iter+1)%max_trajectory_queue_size_;
    }
  }
//  ROS_INFO("No match found");
  return false;
}


void JointTrajectoryController::setTrajectoryCmdFromMsg(manipulation_msgs::JointTraj traj_msg, int id)
{
  std::vector<trajectory::Trajectory::TPoint> tp;
  int msg_size = std::max<int>((int)traj_msg.get_points_size(),1);

  tp.resize(msg_size+1);

  //set first point in trajectory to current position and velocity of the joints
  tp[0].setDimension((int) num_joints_);

  for(int j=0; j < num_joints_; j++)
  {
    tp[0].q_[j] = current_joint_position_[j];
    tp[0].qdot_[j] = current_joint_velocity_[j];
    tp[0].time_ = 0.0;
  }

  if((int)traj_msg.get_points_size() > 0)
  {
    if((int) traj_msg.points[0].get_positions_size() != (int) num_joints_)
    {
      ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) traj_msg.points[0].get_positions_size(), (int) num_joints_);
      return;
    }
    else
    {
      for(int i=0; i < (int) traj_msg.get_points_size(); i++)
      {
        tp[i+1].setDimension((int) num_joints_);
        for(int j=0; j < (int) num_joints_; j++)
        {
          tp[i+1].q_[j] = traj_msg.points[i].positions[j];
          tp[i+1].time_ = traj_msg.points[i].time;
//          ROS_INFO("Trajectory: %d %d %f %f",i,j,tp[i+1].q_[j],tp[i+1].time_);
        }
      }
    }
  }
  else
  {
//    ROS_WARN("Trajectory message has no way points");
    //set second point in trajectory to current position of the arm
    tp[1].setDimension((int) num_joints_);

    for(int j=0; j < num_joints_; j++)
    {
      tp[1].q_[j] = current_joint_position_[j];
      tp[1].time_ = 0.0;
    }
  }

  if(setTrajectoryCmd(tp))
  {
//    ROS_INFO("Setting trajectory command");
    current_trajectory_id_ = id;
    joint_trajectory_status_[current_trajectory_id_] = JointTrajectoryController::ACTIVE;
  }
}

void JointTrajectoryController::TrajectoryReceivedOnTopic(const manipulation_msgs::JointTrajConstPtr &msg)
{
  // PREEMPTS anything on the queue
  this->ros_lock_.lock();
  traj_msg_ = *msg;
  last_traj_req_time_ = current_time_;
//  ROS_INFO("Locked and setting command");
  preemptTrajectoryQueue(traj_msg_,request_trajectory_id_);
  request_trajectory_id_++;
  this->ros_lock_.unlock();
}


bool JointTrajectoryController::setJointTrajSrv(experimental_controllers::TrajectoryStart::Request &req,
                                                experimental_controllers::TrajectoryStart::Response &resp)
{
  addTrajectoryToQueue(req.traj, request_trajectory_id_);
  last_traj_req_time_ = current_time_;
  resp.trajectoryid = request_trajectory_id_;

  if(req.requesttiming)
  {
    trajectory::Trajectory tmp(num_joints_);
    std::vector<double> timestamps;

    createTrajectoryFromMsg(req.traj,tmp);
    resp.set_timestamps_size((int)req.traj.get_points_size());
    timestamps.resize((int)req.traj.get_points_size());

    tmp.getTimeStamps(timestamps);

    for(int i=0; i < (int) req.traj.get_points_size(); i++)
    {
      resp.timestamps[i] = timestamps[i];
    }
  }
  request_trajectory_id_++;
  return true;
}

bool JointTrajectoryController::queryJointTrajSrv(experimental_controllers::TrajectoryQuery::Request &req,
                                                  experimental_controllers::TrajectoryQuery::Response &resp)
{
  resp.set_jointnames_size(num_joints_);
  resp.set_jointpositions_size(num_joints_);
  for(int i=0; i < num_joints_; i++)
  {
    resp.jointnames[i] = joint_name_[i];
    resp.jointpositions[i] = current_joint_position_[i];
    ROS_DEBUG("Joint name: %s",joint_name_[i].c_str());
  }
  if(req.trajectoryid >= (int)joint_trajectory_status_.size() || req.trajectoryid == experimental_controllers::TrajectoryQuery::Request::Query_Joint_Names)
  {
    resp.trajectorytime = 0.0;
    resp.done = JointTrajectoryController::DOES_NOT_EXIST;
    ROS_DEBUG("Query joint traj with id: %d, status: DOES_NOT_EXIST, size of status vector: %zu",req.trajectoryid,joint_trajectory_status_.size());
    return true;
  }
  else
  {
    resp.done = joint_trajectory_status_[req.trajectoryid];
  }

  if(current_trajectory_id_ == (int)req.trajectoryid)
  {
    if((int) resp.done == JointTrajectoryController::DONE)
      resp.trajectorytime = (trajectory_end_time_ - trajectory_start_time_).toSec();
    else
      resp.trajectorytime = (current_time_ - trajectory_start_time_).toSec();
  }
  else
  {
    resp.trajectorytime = joint_trajectory_time_[req.trajectoryid];
  }
  return true;
}

bool JointTrajectoryController::cancelJointTrajSrv(experimental_controllers::TrajectoryCancel::Request &req,
                                                   experimental_controllers::TrajectoryCancel::Response &resp)
{
  int status = JointTrajectoryController::NUM_STATUS;
  std::vector<trajectory::Trajectory::TPoint> trajectory_points_vector;
  if(req.trajectoryid > joint_trajectory_status_.size())
  {
    return false;
  }
  else
  {
    status = joint_trajectory_status_[req.trajectoryid];
  }
  deleteTrajectoryFromQueue(req.trajectoryid);
  return true;
}


bool JointTrajectoryController::createTrajectoryPointsVectorFromMsg(const manipulation_msgs::JointTraj &new_traj, std::vector<trajectory::Trajectory::TPoint> &tp)
{
  if(new_traj.get_points_size() > 0)
  {
    if((int) new_traj.points[0].get_positions_size() != (int) num_joints_)
    {
      ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) new_traj.points[0].get_positions_size(), (int) num_joints_);
      return false;
    }
  }
  else
  {
    ROS_WARN("Trajectory message has no way points");
    return false;
  }

  tp.resize((int)new_traj.get_points_size());

  for(int i=0; i < (int) new_traj.get_points_size(); i++)
  {
    tp[i].setDimension((int) num_joints_);
    for(int j=0; j < (int) num_joints_; j++)
    {
      tp[i].q_[j] = new_traj.points[i].positions[j];
      tp[i].time_ = new_traj.points[i].time;
    }
  }

  return true;
}

bool JointTrajectoryController::createTrajectoryFromMsg(const manipulation_msgs::JointTraj &new_traj,trajectory::Trajectory &return_trajectory)
{
  std::vector<trajectory::Trajectory::TPoint> tp;

  if(!createTrajectoryPointsVectorFromMsg(new_traj, tp))
  {
    return false;
  }

  return_trajectory.setMaxRates(joint_velocity_limits_);
  return_trajectory.setInterpolationMethod(trajectory_type_);

  if(!return_trajectory.setTrajectory(tp))
  {
    ROS_WARN("Trajectory not set correctly");
    return false;
  }
  return true;
}

void JointTrajectoryController::addTrajectoryToQueue(manipulation_msgs::JointTraj new_traj, int id)
{
  trajectory_queue_.lock();

  joint_trajectory_vector_[next_free_index_] = new_traj;
  joint_trajectory_id_[next_free_index_] = id;
  joint_trajectory_time_.push_back(0.0);
  joint_trajectory_status_.push_back(JointTrajectoryController::QUEUED);
  num_trajectory_available_ = (num_trajectory_available_+1)%max_trajectory_queue_size_;
  next_free_index_ = (next_free_index_+1)%max_trajectory_queue_size_;
  trajectory_queue_.unlock();
}

void JointTrajectoryController::preemptTrajectoryQueue(manipulation_msgs::JointTraj new_traj, int id)
{
  int index = std::max(current_trajectory_index_,0);
  trajectory_queue_.try_lock();
//  ROS_INFO("Setting preempt command");
  joint_trajectory_vector_[index] = new_traj;
  joint_trajectory_id_[index] = id;
  joint_trajectory_time_.push_back(0.0);
  joint_trajectory_status_.push_back(JointTrajectoryController::QUEUED);
  num_trajectory_available_ = (num_trajectory_available_+1)%max_trajectory_queue_size_;
  trajectory_queue_.unlock();

  trajectory_preempted_ = true;
//  ROS_INFO("Done setting preempt command");
}

void JointTrajectoryController::deleteTrajectoryFromQueue(int id)
{
  trajectory_queue_.lock();
  if(id == joint_trajectory_id_[current_trajectory_index_])
  {
    joint_trajectory_vector_[current_trajectory_index_].set_points_size(0);
    trajectory_queue_.unlock();
    trajectory_preempted_ = true;
    return;
  }

  if(num_trajectory_available_ < 1)
    return;

// do a linear search from current index + 1
  int iter = (current_trajectory_index_ + 1)%max_trajectory_queue_size_;
  int num_iterations = 0;
  while(num_iterations < max_trajectory_queue_size_)
  {
    if(joint_trajectory_id_[iter] == id)
    {
      joint_trajectory_status_[id] = JointTrajectoryController::DELETED;
      num_trajectory_available_--;
      break;
    }
    num_iterations++;
    iter = (iter+1)%max_trajectory_queue_size_;
  }
  trajectory_queue_.unlock();
}

void JointTrajectoryController::publishDiagnostics()
{
  if(!((current_time_ - last_diagnostics_publish_time_).toSec() > diagnostics_publish_delta_time_))
  {
    return;
  }

  if(diagnostics_publisher_->trylock())
  {
    deprecated_msgs::JointCmd cmd;
    cmd.set_names_size(1);
    cmd.set_positions_size(1);
    cmd.set_velocity_size(1);

    vector<diagnostic_msgs::DiagnosticStatus> statuses;
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = "Whole Body Trajectory Controller";
    if(watch_dog_active_)
    {
      status.summary(0, "WATCHDOG");
    }
    else
    {
      status.summary(0, "OK");
    }

    for(unsigned int i=0; i < joint_pv_controllers_.size(); i++)
    {
      status.add(joint_pv_controllers_[i]->getJointName() + "/Position/Actual",
          joint_pv_controllers_[i]->joint_state_->position_);
      joint_pv_controllers_[i]->getCommand(cmd);
      status.add(joint_pv_controllers_[i]->getJointName() + "/Position/Command",
          cmd.positions[0]);
      status.add(joint_pv_controllers_[i]->getJointName() + "/Position/Error (Command-Actual)",
          cmd.positions[0] - joint_pv_controllers_[i]->joint_state_->position_);
    }

    status.add("Trajectory id", current_trajectory_id_);

    std::string key = "Trajectory Status:: ";

    if(current_trajectory_id_ < 0)
    {
      status.add(key + "AT REST", -1);
    }
    else
    {
      status.add(key + JointTrajectoryStatusString[joint_trajectory_status_[current_trajectory_id_]],
          joint_trajectory_status_[current_trajectory_id_]);
    }

    status.add("Trajectory Current Time", current_time_-trajectory_start_time_);
    status.add("Trajectory Expected End Time (computed)",
        trajectory_end_time_-trajectory_start_time_);
    status.add("Current trajectory queue index", current_trajectory_index_);
    status.add("Number queued trajectories", num_trajectory_available_);
    status.add("Next queue free index", next_free_index_);
    status.add("Number joints", num_joints_);
    status.add("Velocity scaling factor", velocity_scaling_factor_);
    status.add("Trajectory wait timeout", trajectory_wait_timeout_);
    status.add("Max allowed update time", max_allowed_update_time_);
    status.add("Diagnostics publish time", diagnostics_publish_delta_time_);
    status.add("Max trajectory queue size", max_trajectory_queue_size_);
    status.add("Listen topic name", listen_topic_name_);
    status.add("Trajectory set service", name_ + "/TrajectoryStart");
    status.add("Trajectory query service", name_ + "/TrajectoryQuery");
    status.add("Trajectory cancel service", name_ + "/TrajectoryCancel");

    statuses.push_back(status);

    last_diagnostics_publish_time_ = current_time_;
    diagnostics_publisher_->msg_.set_status_vec(statuses);
    diagnostics_publisher_->unlockAndPublish();
  }
}
