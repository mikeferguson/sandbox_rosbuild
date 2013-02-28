/* 
 * Copyright (c) 2011, Vanadium Labs LLC
 * All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Vanadium Labs LLC nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: Michael Ferguson
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>

#include <vector>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
tf::TransformListener * tf_listener_;
geometry_msgs::Pose last_pose_;

void frameCallback(const ros::TimerEvent&)
{
    static tf::TransformBroadcaster br;
    tf::Transform t;
    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(last_pose_.position.x,last_pose_.position.y,last_pose_.position.z));
    t.setRotation(tf::Quaternion(last_pose_.orientation.x, last_pose_.orientation.y, last_pose_.orientation.z, last_pose_.orientation.w));
    br.sendTransform(tf::StampedTransform(t, time, "head_tilt_link", "head_sensor_frame"));
}


/* 
 * Print new location
 */
void update( const InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        {
        double roll, pitch, yaw;
        btQuaternion q(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);
        btMatrix3x3 m(q);        
        m.getRPY(roll, pitch, yaw);
        ROS_INFO_STREAM( "xyz: " << feedback->pose.position.x << ", " 
                         << feedback->pose.position.y << ", " 
                         << feedback->pose.position.z << ", "
                         << "rpy: " << roll << ", "
                         << pitch << ", "
                         << yaw << "\n" );
        }
        break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        last_pose_.position.x = feedback->pose.position.x;
        last_pose_.position.y = feedback->pose.position.y;
        last_pose_.position.z = feedback->pose.position.z;
        last_pose_.orientation.x = feedback->pose.orientation.x;
        last_pose_.orientation.y = feedback->pose.orientation.y;
        last_pose_.orientation.z = feedback->pose.orientation.z;
        last_pose_.orientation.w = feedback->pose.orientation.w;
        break;
  }
  
  server->applyChanges(); 
}

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "interactive_camera_pose");
  ros::NodeHandle nh;
  ros::Timer frame_timer = nh.createTimer(ros::Duration(0.1), frameCallback);

  last_pose_.position.x = 0;
  last_pose_.position.y = 0;
  last_pose_.position.z = 0;
  last_pose_.orientation.x = 0;
  last_pose_.orientation.y = 0;
  last_pose_.orientation.z = 0;
  last_pose_.orientation.w = 1;

  // create marker server
  server.reset( new interactive_markers::InteractiveMarkerServer("camera_control","",false) );

  InteractiveMarker marker;
  marker.header.frame_id = "/head_tilt_link";
  marker.scale = 1;

  marker.name = "camera_pose";
  marker.description = "Camera Pose Controller";

  // insert a box
  makeBoxControl(marker);
  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  server->insert(marker);
  server->setCallback(marker.name, &update);

  server->applyChanges();

  // everything is done in cloud callback, just spin
  ros::spin();
  
  server.reset();  
}



