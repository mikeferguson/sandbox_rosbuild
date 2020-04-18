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

#include "experimental_controllers/arm_trajectory_controller.h"
#include "angles/angles.h"
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(ArmTrajectoryControllerNode, controller::ArmTrajectoryControllerNode, pr2_controller_interface::Controller)

using namespace controller;
using namespace std;



static const std::string JointTrajectoryStatusString[7] = {"0 - ACTIVE","1 - DONE","2 - QUEUED","3 - DELETED","4 - FAILED","5 - CANCELED","6 - NUM_STATUS"};

ArmTrajectoryController::ArmTrajectoryController() :
  refresh_rt_vals_(false),trajectory_type_("linear"),trajectory_wait_time_(0.0), max_update_time_(0.0)
{
  controller_state_publisher_ = NULL;
}

ArmTrajectoryController::~ArmTrajectoryController()
{
  // Assumes the joint controllers are owned by this class.
  for(unsigned int i=0; i<joint_pd_controllers_.size();++i)
    delete joint_pd_controllers_[i];
}

bool ArmTrajectoryController::initXml(pr2_mechanism_model::RobotState * robot, TiXmlElement * config)
{
  ROS_INFO("Initializing trajectory controller");

  robot_ = robot->model_;
  robot_state_ = robot;
  pr2_mechanism_model::JointState *js;
//  std::vector<double> joint_velocity_limits;
  std::vector<trajectory::Trajectory::TPoint> trajectory_points_vector;
//  std::string trajectory_type = "linear";

  TiXmlElement *elt = config->FirstChildElement("controller");
  while (elt)
  {
    JointPDController * jpc = new JointPDController();
//    std::cout<<elt->Attribute("type")<<elt->Attribute("name")<<std::endl;
    ROS_DEBUG("Joint Position Controller: %s , %s",(elt->Attribute("type")),(elt->Attribute("name")));
    assert(static_cast<std::string>(elt->Attribute("type")) == std::string("JointPDController"));

    joint_pd_controllers_.push_back(jpc);
    if(!jpc->initXml(robot, elt))
      return false;

    js = (robot->getJointState(jpc->getJointName()));
    if(js && js->joint_->limits)
    {
      joint_velocity_limits_.push_back(js->joint_->limits->velocity*velocity_scaling_factor_);
      joint_type_.push_back(js->joint_->type);
      if(js->joint_->type == urdf::Joint::CONTINUOUS)
        ROS_INFO("Pushing back joint of type: continuous joint: %s",js->joint_->name.c_str());
    }

    elt = elt->NextSiblingElement("controller");
  }


  elt = config->FirstChildElement("trajectory");

  if(!elt)
    ROS_WARN("No trajectory information in xml file. ");
  else
  {
    trajectory_type_ = std::string(elt->Attribute("interpolation"));
    ROS_DEBUG("ArmTrajectoryController:: interpolation type:: %s",trajectory_type_.c_str());
  }
  joint_cmd_rt_.resize(joint_pd_controllers_.size());
  joint_cmd_dot_rt_.resize(joint_pd_controllers_.size());

  joint_trajectory_ = new trajectory::Trajectory((int) joint_pd_controllers_.size());

  joint_trajectory_->setMaxRates(joint_velocity_limits_);
  joint_trajectory_->setInterpolationMethod(trajectory_type_);

  trajectory_point_.setDimension((int) joint_pd_controllers_.size());
  dimension_ = (int) joint_pd_controllers_.size();

  for(int i=0; i < dimension_; i++)
  {
    if(joint_type_[i] == pr2_mechanism_model::JOINT_CONTINUOUS)
    {
      joint_trajectory_->setJointWraps(i);
    }
  }

// Add two points since every good trajectory must have at least two points, otherwise its just a point :-)
  for(int j=0; j < dimension_; j++)
    trajectory_point_.q_[j] = joint_pd_controllers_[j]->joint_state_->position_;
  trajectory_point_.time_ = 0.0;
  trajectory_points_vector.push_back(trajectory_point_);

  for(int i=0; i < dimension_; i++)
     trajectory_point_.q_[i] = joint_pd_controllers_[i]->joint_state_->position_;
  trajectory_point_.time_ = 0.0;
  trajectory_points_vector.push_back(trajectory_point_);

  joint_trajectory_->autocalc_timing_ = true;

  ROS_DEBUG("Size of trajectory points vector : %zu",trajectory_points_vector.size());
  if(!joint_trajectory_->setTrajectory(trajectory_points_vector))
    ROS_WARN("Trajectory not set correctly");

  trajectory_start_time_ = robot_state_->getTime();
  trajectory_end_time_ = trajectory_start_time_;

  ROS_INFO("ArmTrajectoryController:: Done loading controller");

  return true;
}

void ArmTrajectoryController::setTrajectoryCmd(const std::vector<trajectory::Trajectory::TPoint>& joint_trajectory)
{
  if(joint_trajectory.size() < 1)
  {
    ROS_WARN("ArmTrajectoryController:: No points in trajectory");
    return;
  }

  joint_trajectory_->setTrajectory(joint_trajectory);
//  joint_trajectory_->write("foo.txt",0.01);
  arm_controller_lock_.lock();
  refresh_rt_vals_ = true;
  arm_controller_lock_.unlock();
}

void ArmTrajectoryController::getJointPosCmd(experimental_controllers::JointPosCmd & cmd) const
{
}

controller::JointPDController* ArmTrajectoryController::getJointControllerByName(std::string name)
{
  for(int i=0; i< (int) joint_pd_controllers_.size(); i++)
  {
    if(joint_pd_controllers_[i]->getJointName() == name)
    {
      return joint_pd_controllers_[i];
    }
  }
    return NULL;
}

int ArmTrajectoryController::getJointControllerPosByName(std::string name)
{
  for(int i=0; i< (int) joint_pd_controllers_.size(); i++)
  {
    if(joint_pd_controllers_[i]->getJointName() == name)
    {
      return i;
    }
  }
  return -1;
}

void ArmTrajectoryController::update(void)
{

#ifdef PUBLISH_MAX_TIME
  double start_time = ros::Time::now().toSec();
#endif

  double sample_time(0.0);

  current_time_ = robot_state_->getTime();

  if(refresh_rt_vals_)
  {
    trajectory_start_time_ = robot_state_->getTime();
    trajectory_wait_time_ = 0.0;
    trajectory_end_time_ = trajectory_start_time_ + ros::Duration(joint_trajectory_->getTotalTime());
    refresh_rt_vals_ = false;
    trajectory_done_ = false;
  }
  if(arm_controller_lock_.try_lock())
  {
    sample_time = (robot_state_->getTime() - trajectory_start_time_).toSec();
    joint_trajectory_->sample(trajectory_point_,sample_time);

//    ROS_INFO("sample_time: %f", sample_time);
    for(unsigned int i=0; i < joint_cmd_rt_.size(); ++i)
    {
      joint_cmd_rt_[i] = trajectory_point_.q_[i];
      joint_cmd_dot_rt_[i] = trajectory_point_.qdot_[i];
//      cout << " " << joint_cmd_rt_[i];
    }
//    cout << endl;
    arm_controller_lock_.unlock();

    if(robot_state_->getTime() >= trajectory_end_time_ && trajectory_done_ == false)
    {
      trajectory_wait_time_ = (robot_state_->getTime() - trajectory_end_time_).toSec();
      if(reachedGoalPosition(joint_cmd_rt_))
      {
        trajectory_wait_time_ = 0.0;
        trajectory_done_= true;
      }
    }
  }

  for(unsigned int i=0;i<joint_pd_controllers_.size();++i)
    joint_pd_controllers_[i]->setCommand(joint_cmd_rt_[i],joint_cmd_dot_rt_[i]);

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

}

bool ArmTrajectoryController::reachedGoalPosition(std::vector<double> joint_cmd)
{
  bool return_val = true;
  double error(0.0);
  for(unsigned int i=0;i< (unsigned int) dimension_;++i){
   if(joint_type_[i] == pr2_mechanism_model::JOINT_CONTINUOUS)
   {
     error = fabs(angles::shortest_angular_distance(joint_pd_controllers_[i]->joint_state_->position_,joint_cmd[i]));
   }
   else
   {
    error = fabs(joint_pd_controllers_[i]->joint_state_->position_ - joint_cmd[i]);
   }
    return_val = return_val && (error <= goal_reached_threshold_[i]);
    //ROS_INFO("joint error: %f threshold: %f",error, goal_reached_threshold_[i]);
  }
  return return_val;
}

void ArmTrajectoryController::updateJointControllers(void)
{
  for(unsigned int i=0;i<joint_pd_controllers_.size();++i)
    joint_pd_controllers_[i]->update();
}

//------ Arm controller node --------


ArmTrajectoryControllerNode::ArmTrajectoryControllerNode()
  : Controller(), request_trajectory_id_(1), current_trajectory_id_(0), trajectory_wait_timeout_(10.0)
{
  ROS_DEBUG("Controller node created");
  c_ = new ArmTrajectoryController();
  diagnostics_publisher_ = NULL;
}

ArmTrajectoryControllerNode::~ArmTrajectoryControllerNode()
{
  c_->controller_state_publisher_->stop();
  delete c_->controller_state_publisher_;

  diagnostics_publisher_->stop();
  delete diagnostics_publisher_;

   if(topic_name_ptr_ && topic_name_.c_str())
  {
    ROS_DEBUG("unsub arm controller %s", topic_name_.c_str());
  }
  delete c_;
}

void ArmTrajectoryControllerNode::update()
{
  if(c_->trajectory_done_)
  {
    updateTrajectoryQueue( ArmTrajectoryControllerNode::DONE);
  }
  if(c_->trajectory_wait_time_ >= trajectory_wait_timeout_)
  {
    updateTrajectoryQueue( ArmTrajectoryControllerNode::FAILED);
  }


  c_->update();
//  ROS_INFO("Publishing diagnostics");
  if((c_->current_time_ - last_diagnostics_publish_time_).toSec() > diagnostics_publish_delta_time_)
  {
    publishDiagnostics();
    last_diagnostics_publish_time_ = c_->current_time_;
  }
//  ROS_INFO("Published diagnostics");
}

void ArmTrajectoryControllerNode::updateTrajectoryQueue(int last_trajectory_finish_status)
{
  if(joint_trajectory_vector_.size() > 0)
  {
    if(current_trajectory_id_ == joint_trajectory_id_.front())
    {
      joint_trajectory_status_[current_trajectory_id_] = last_trajectory_finish_status;
      joint_trajectory_time_[current_trajectory_id_] = (c_->trajectory_end_time_ - c_->trajectory_start_time_).toSec();
      joint_trajectory_vector_.erase(joint_trajectory_vector_.begin());
      joint_trajectory_id_.erase(joint_trajectory_id_.begin());
    }
    if(joint_trajectory_vector_.size() > 0)
    {
      setTrajectoryCmdFromMsg(joint_trajectory_vector_.front());
      current_trajectory_id_ = joint_trajectory_id_.front();
      joint_trajectory_status_[current_trajectory_id_] = ArmTrajectoryControllerNode::ACTIVE;
    }
    else
    {
      current_trajectory_id_ = 0;
    }
  }
}


bool ArmTrajectoryControllerNode::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
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

bool ArmTrajectoryControllerNode::initXml(pr2_mechanism_model::RobotState * robot, TiXmlElement * config)
{
  ROS_INFO("Loading ArmTrajectoryControllerNode.");
  service_prefix_ = config->Attribute("name");
  ROS_DEBUG("The service_prefix_ is %s",service_prefix_.c_str());

  double scale;
  node_.param<double>(service_prefix_ + "/velocity_scaling_factor",scale,0.25);
  node_.param<double>(service_prefix_ + "/trajectory_wait_timeout",trajectory_wait_timeout_,10.0);

  ROS_DEBUG("Trajectory wait timeout scale is %f",scale);
  c_->velocity_scaling_factor_ = std::min(1.0,std::max(0.0,scale));

  ROS_DEBUG("Velocity scaling factor is %f",c_->velocity_scaling_factor_);
  ROS_DEBUG("Trajectory wait timeout is %f",trajectory_wait_timeout_);

  if(c_->initXml(robot, config))  // Parses subcontroller configuration
  {

   trajectory_start_srv_ = node_.advertiseService(service_prefix_ + "/TrajectoryStart", &ArmTrajectoryControllerNode::setJointTrajSrv, this);
   trajectory_query_srv_ = node_.advertiseService(service_prefix_ + "/TrajectoryQuery", &ArmTrajectoryControllerNode::queryJointTrajSrv, this);
   trajectory_cancel_srv_= node_.advertiseService(service_prefix_ + "/TrajectoryCancel", &ArmTrajectoryControllerNode::cancelJointTrajSrv, this);

    topic_name_ptr_ = config->FirstChildElement("listen_topic");
    if(topic_name_ptr_)
    {
      topic_name_= topic_name_ptr_->Attribute("name");
      if(!topic_name_.c_str())
      {
        ROS_ERROR(" A listen _topic is present in the xml file but no name is specified");
        return false;
      }
      trajectory_cmd_sub_ = node_.subscribe(topic_name_, 1, &ArmTrajectoryControllerNode::CmdTrajectoryReceived, this);
      ROS_DEBUG("Listening to topic: %s",topic_name_.c_str());
    }

    getJointTrajectoryThresholds();


  if (c_->controller_state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete c_->controller_state_publisher_ ;
  c_->controller_state_publisher_ = new realtime_tools::RealtimePublisher <experimental_controllers::ControllerState> (service_prefix_+"/controller_state", 1) ;

  if (diagnostics_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete diagnostics_publisher_ ;
  diagnostics_publisher_ = new realtime_tools::RealtimePublisher <diagnostic_msgs::DiagnosticArray> ("/diagnostics", 2) ;

  last_diagnostics_publish_time_ = c_->robot_state_->getTime();
  node_.param<double>(service_prefix_ + "/diagnostics_publish_delta_time",diagnostics_publish_delta_time_,0.05);

  ROS_DEBUG("Initialized publisher");

  c_->controller_state_publisher_->msg_.name = std::string(service_prefix_);

    ROS_INFO("Initialized controller");
    return true;
  }
  ROS_ERROR("Could not initialize controller");
  return false;
}

void ArmTrajectoryControllerNode::getJointTrajectoryThresholds()
{
  c_->goal_reached_threshold_.resize(c_->dimension_);
  for(int i=0; i< c_->dimension_;i++)
  {
    node_.param<double>(service_prefix_ + "/" + c_->joint_pd_controllers_[i]->getJointName() + "/goal_reached_threshold",c_->goal_reached_threshold_[i],GOAL_REACHED_THRESHOLD);
    ROS_DEBUG("Goal distance threshold for %s is %f",c_->joint_pd_controllers_[i]->getJointName().c_str(),c_->goal_reached_threshold_[i]);
  }
}

void ArmTrajectoryControllerNode::setTrajectoryCmdFromMsg(manipulation_msgs::JointTraj traj_msg)
{
  std::vector<trajectory::Trajectory::TPoint> tp;
  int msg_size = std::max<int>((int)traj_msg.get_points_size(),1);

  tp.resize(msg_size+1);

  //set first point in trajectory to current position of the arm
  tp[0].setDimension((int) c_->dimension_);

  for(int j=0; j < c_->dimension_; j++)
  {
    tp[0].q_[j] = c_->joint_pd_controllers_[j]->joint_state_->position_;
    tp[0].time_ = 0.0;
  }

  if((int)traj_msg.get_points_size() > 0)
  {
    if((int) traj_msg.points[0].get_positions_size() != (int) c_->dimension_)
    {
      ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) traj_msg.points[0].get_positions_size(), (int) c_->dimension_);
      return;
    }
    else
    {
      for(int i=0; i < (int) traj_msg.get_points_size(); i++)
      {
        tp[i+1].setDimension((int) c_->dimension_);
        for(int j=0; j < (int) c_->dimension_; j++)
        {
          tp[i+1].q_[j] = traj_msg.points[i].positions[j];
          tp[i+1].time_ = traj_msg.points[i].time;
        }
      }
    }
  }
  else
  {
    ROS_WARN("Trajectory message has no way points");
    //set second point in trajectory to current position of the arm
    tp[1].setDimension((int) c_->dimension_);

    for(int j=0; j < c_->dimension_; j++)
    {
      tp[1].q_[j] = c_->joint_pd_controllers_[j]->joint_state_->position_;
      tp[1].time_ = 0.0;
    }
  }

  this->c_->setTrajectoryCmd(tp);
}

void ArmTrajectoryControllerNode::CmdTrajectoryReceived(const manipulation_msgs::JointTrajConstPtr &msg)
{
  ROS_DEBUG("Trajectory controller:: Cmd received");
  this->ros_lock_.lock();
  traj_msg_ = *msg;
  setTrajectoryCmdFromMsg(traj_msg_);
  this->ros_lock_.unlock();
}


bool ArmTrajectoryControllerNode::getJointPosCmd(experimental_controllers::GetJointPosCmd::Request &req,
                    experimental_controllers::GetJointPosCmd::Response &resp)
{
  experimental_controllers::JointPosCmd cmd;
  c_->getJointPosCmd(cmd);
  resp.command = cmd;
  return true;
}


bool ArmTrajectoryControllerNode::setJointTrajSrv(experimental_controllers::TrajectoryStart::Request &req,
                    experimental_controllers::TrajectoryStart::Response &resp)
{
  addTrajectoryToQueue(req.traj, request_trajectory_id_);
  resp.trajectoryid = request_trajectory_id_;

  if(req.requesttiming)
  {
    trajectory::Trajectory tmp(c_->dimension_);
    createTrajectory(req.traj,tmp);
    std::vector<double> timestamps;
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

bool ArmTrajectoryControllerNode::queryJointTrajSrv(experimental_controllers::TrajectoryQuery::Request &req,
                                                    experimental_controllers::TrajectoryQuery::Response &resp)
{
  resp.set_jointnames_size(c_->dimension_);
  resp.set_jointpositions_size(c_->dimension_);
  for(int i=0; i < c_->dimension_; i++)
  {
    resp.jointnames[i] = c_->joint_pd_controllers_[i]->getJointName();
    resp.jointpositions[i] = c_->joint_pd_controllers_[i]->joint_state_->position_;
  }

  if(req.trajectoryid == 0)
  {
    resp.trajectorytime = 0;
    if(joint_trajectory_vector_.size() == 0)
      resp.done = 1;
    else
      resp.done = 0;
    return true;
  }

  std::map<int, int>::const_iterator it = joint_trajectory_status_.find((int)req.trajectoryid);
  if(it == joint_trajectory_status_.end())
    return false;
  else
    resp.done = it->second;


  if(current_trajectory_id_ == (int)req.trajectoryid)
  {
    if((int) resp.done == 1)
      resp.trajectorytime = (c_->trajectory_end_time_ - c_->trajectory_start_time_).toSec();
    else
      resp.trajectorytime = (c_->current_time_ - c_->trajectory_start_time_).toSec();
  }
  else
  {
    std::map<int, double>::const_iterator it_time = joint_trajectory_time_.find((int)req.trajectoryid);
    if(it_time == joint_trajectory_time_.end())
      return false;

    resp.trajectorytime = it_time->second;
  }
  return true;
}

bool ArmTrajectoryControllerNode::cancelJointTrajSrv(experimental_controllers::TrajectoryCancel::Request &req,
                                                     experimental_controllers::TrajectoryCancel::Response &resp)
{
  int status = ArmTrajectoryControllerNode::NUM_STATUS;

  std::vector<trajectory::Trajectory::TPoint> trajectory_points_vector;
  std::map<int, int>::const_iterator it = joint_trajectory_status_.find((int)req.trajectoryid);

  if(it == joint_trajectory_status_.end())
    return false;
  else
    status = it->second;

  if(status == ArmTrajectoryControllerNode::QUEUED)
  {
    deleteTrajectoryFromQueue(req.trajectoryid);
  }
  else if(status == ArmTrajectoryControllerNode::ACTIVE)
  {
    updateTrajectoryQueue(CANCELED);
    // Add two points since every good trajectory must have at least two points, otherwise its just a point :-)
    for(int j=0; j < c_->dimension_; j++)
      c_->trajectory_point_.q_[j] = c_->joint_pd_controllers_[j]->joint_state_->position_;
    c_->trajectory_point_.time_ = 0.0;
    trajectory_points_vector.push_back(c_->trajectory_point_);

    for(int i=0; i < c_->dimension_; i++)
      c_->trajectory_point_.q_[i] =  c_->joint_pd_controllers_[i]->joint_state_->position_;
    c_->trajectory_point_.time_ = 0.0;
    trajectory_points_vector.push_back(c_->trajectory_point_);
    if(!c_->joint_trajectory_->setTrajectory(trajectory_points_vector))
      ROS_WARN("Trajectory not set correctly");
  }
  return true;
}

int ArmTrajectoryControllerNode::createTrajectory(const manipulation_msgs::JointTraj &new_traj,trajectory::Trajectory &return_trajectory)
{
  std::vector<trajectory::Trajectory::TPoint> tp;

  if(new_traj.get_points_size() > 0)
  {
     if((int) new_traj.points[0].get_positions_size() != (int) c_->dimension_)
     {
       ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) new_traj.points[0].get_positions_size(), (int) c_->dimension_);
        return -1;
     }
  }
  else
  {
    ROS_WARN("Trajectory message has no way points");
     return -1;
  }

  tp.resize((int)new_traj.get_points_size());

  for(int i=0; i < (int) new_traj.get_points_size(); i++)
  {
     tp[i].setDimension((int) c_->dimension_);
     for(int j=0; j < (int) c_->dimension_; j++)
     {
        tp[i].q_[j] = new_traj.points[i].positions[j];
        tp[i].time_ = new_traj.points[i].time;
     }
  }
  return_trajectory.setMaxRates(c_->joint_velocity_limits_);
  return_trajectory.setInterpolationMethod(c_->trajectory_type_);

  if(!return_trajectory.setTrajectory(tp))
  {
    ROS_WARN("Trajectory not set correctly");
    return -1;
  }
  return 1;
}

void ArmTrajectoryControllerNode::addTrajectoryToQueue(manipulation_msgs::JointTraj new_traj, int id)
{
  joint_trajectory_vector_.push_back(new_traj);
  joint_trajectory_id_.push_back(id);
  joint_trajectory_status_[id] = ArmTrajectoryControllerNode::QUEUED;
  joint_trajectory_time_[id] = 0.0;
}

void ArmTrajectoryControllerNode::deleteTrajectoryFromQueue(int id)
{
// do a linear search
  for(int i = 0; i < (int) joint_trajectory_vector_.size(); i++)
  {
    if(joint_trajectory_id_[i] == id)
    {
      joint_trajectory_vector_.erase(joint_trajectory_vector_.begin()+i);
      joint_trajectory_id_.erase(joint_trajectory_id_.begin()+i);
      joint_trajectory_status_[id] = ArmTrajectoryControllerNode::DELETED;
      break;
    }
  }
}


void ArmTrajectoryControllerNode::publishDiagnostics()
{
//  ROS_INFO("Starting diagnostics");
  if(diagnostics_publisher_->trylock())
  {
//    ROS_INFO("Started diagnostics");
    deprecated_msgs::JointCmd cmd;
    cmd.set_names_size(1);
    cmd.set_positions_size(1);
    cmd.set_velocity_size(1);

    vector<diagnostic_msgs::DiagnosticStatus> statuses;
    diagnostic_updater::DiagnosticStatusWrapper status;

    status.name = service_prefix_;
    status.summary(0, "OK");

//    ROS_INFO("Diagnostics 1");

    for(unsigned int i=0; i < c_->joint_pd_controllers_.size(); i++)
    {
      status.add(c_->joint_pd_controllers_[i]->getJointName() + "/Position/Actual",
          c_->joint_pd_controllers_[i]->joint_state_->position_);

//      ROS_INFO("Diagnostics %d: 1",i);

      c_->joint_pd_controllers_[i]->getCommand(cmd);
      status.add(c_->joint_pd_controllers_[i]->getJointName() + "/Position/Command",
          cmd.positions[0]);

      status.add(c_->joint_pd_controllers_[i]->getJointName() + "/Position/Error (Command-Actual)",
      cmd.positions[0] - c_->joint_pd_controllers_[i]->joint_state_->position_);

//      ROS_INFO("Diagnostics %d: 2",i);

    }

    status.add("Trajectory id", current_trajectory_id_);

//    ROS_INFO("Diagnostics 2");

    std::string key = "Trajectory Status:: ";
    std::map<int, int>::const_iterator it = joint_trajectory_status_.find((int)current_trajectory_id_);
    if(it == joint_trajectory_status_.end())
      status.add(key + "UNKNOWN", -1);
    else
      status.add(key + JointTrajectoryStatusString[it->second], it->second);

//    ROS_INFO("Diagnostics 3");

    status.add("Trajectory Current Time",
        c_->current_time_-c_->trajectory_start_time_);

//    ROS_INFO("Diagnostics 4");

    status.add("Trajectory Expected End Time (computed)",
        c_->trajectory_end_time_-c_->trajectory_start_time_);

//    ROS_INFO("Diagnostics 5");

    statuses.push_back(status);
    diagnostics_publisher_->msg_.set_status_vec(statuses);
//    ROS_INFO("Set diagnostics info");
    diagnostics_publisher_->unlockAndPublish();
  }
}
