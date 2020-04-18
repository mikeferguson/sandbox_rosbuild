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

#include <ros/node.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include <pr2_mechanism_controllers/TrajectoryCancel.h>

static int done = 0;

void finalize(int donecare)
{
  done = 1;
}

int main( int argc, char** argv )
{

  /*********** Initialize ROS  ****************/
  ros::init(argc,argv);
  ros::Node *node = new ros::Node("arm_trajectory_controller_client"); 

  signal(SIGINT,  finalize);
  signal(SIGQUIT, finalize);
  signal(SIGTERM, finalize);

  pr2_mechanism_controllers::TrajectoryStart::Request  req;
  pr2_mechanism_controllers::TrajectoryStart::Response res;

  pr2_mechanism_controllers::TrajectoryQuery::Request  req_q;
  pr2_mechanism_controllers::TrajectoryQuery::Response res_q;

  pr2_mechanism_controllers::TrajectoryCancel::Request  rec_req;
  pr2_mechanism_controllers::TrajectoryCancel::Response rec_res;

  int num_points = 3;
  int num_joints = 7;

  req.traj.set_points_size(num_points);
  req.requesttiming = 1;

  for(int i=0; i<num_points; i++)
    req.traj.points[i].set_positions_size(num_joints);


  req.traj.points[0].positions[0] = 0.5;
  req.traj.points[0].positions[1] = 0.5;
  req.traj.points[0].positions[2] = 0.2;
  req.traj.points[0].positions[3] = -0.5;
  req.traj.points[0].positions[4] = 0.4;
  req.traj.points[0].positions[5] = 0.0;
  req.traj.points[0].positions[6] = 0.0;
  req.traj.points[0].time = 0.0;

  req.traj.points[1].positions[0] = 0.0;
  req.traj.points[1].positions[1] = 0.0;
  req.traj.points[1].positions[2] = 0.0;
  req.traj.points[1].positions[3] = 0.0;
  req.traj.points[1].positions[4] = 0.0;
  req.traj.points[1].positions[5] = 0.0;
  req.traj.points[1].positions[6] = 0.0;
  req.traj.points[1].time = 0.0;

  req.traj.points[2].positions[0] = -0.5;
  req.traj.points[2].positions[1] = 0.3;
  req.traj.points[2].positions[2] = 0.2;
  req.traj.points[2].positions[3] = -1.0;
  req.traj.points[2].positions[4] = -0.4;
  req.traj.points[2].positions[5] = 0.0;
  req.traj.points[2].positions[6] = 0.0;
  req.traj.points[2].time = 0.0;


  int num_tries = 0;

  if (ros::service::call("r_arm_joint_trajectory_controller/TrajectoryStart", req, res))
  {
    ROS_INFO("Done");
  }
//  sleep(0.1);

  req_q.trajectoryid = res.trajectoryid;
  rec_req.trajectoryid = res.trajectoryid;
  
  if(ros::service::call("r_arm_joint_trajectory_controller/TrajectoryCancel", rec_req, rec_res))  
  {
  }
  else
  {
    ROS_INFO("service call failed");
  }


  usleep(1e4);
  if(ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q))  
  {
    while(res_q.done != res_q.State_Deleted && res_q.done != res_q.State_Canceled)
    {
      ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q);
      usleep(1e5);
    }
  
    ROS_INFO("response 1:: id:%d, time:%f, response:%d",req_q.trajectoryid,res_q.trajectorytime,res_q.done);
  }
  else
  {
    ROS_INFO("service call failed");
  }

  usleep(1e4);
  req_q.trajectoryid =  pr2_mechanism_controllers::TrajectoryQuery::Request::Query_Joint_Names;

  if(ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q))  
  {
    ROS_INFO("response 2:: %f, %d",res_q.trajectorytime,res_q.done);
  }
  else
  {
    ROS_INFO("service call failed");
    for(int i=0; i < (int) res_q.jointnames.size(); i++)
    {
      ROS_INFO("Joint name: %s", res_q.jointnames[i].c_str());
    }      
  }

  if (ros::service::call("r_arm_joint_trajectory_controller/TrajectoryStart", req, res))
  {
    ROS_INFO("Done");
  }

  req_q.trajectoryid = res.trajectoryid;
  rec_req.trajectoryid = res.trajectoryid;

  sleep(4.0);
  
  if(ros::service::call("r_arm_joint_trajectory_controller/TrajectoryCancel", rec_req, rec_res))  
  {
  }
  else
  {
    ROS_INFO("service call failed");
  }

  if(ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q))  
  {
    while(res_q.done != res_q.State_Deleted && res_q.done != res_q.State_Canceled)
    {
      ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q);
      usleep(1e5);
    }  
    ROS_INFO("response 3:: id:%d, time:%f, response:%d",req_q.trajectoryid,res_q.trajectorytime,res_q.done);
  }
  else
  {
    ROS_INFO("service call failed");
  }
}
