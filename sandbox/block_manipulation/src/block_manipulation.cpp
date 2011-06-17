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
#include <sensor_msgs/PointCloud2.h>
#include <simple_arm_server/MoveArm.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/transforms.h>

#include <interactive_markers/interactive_marker_server.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::ServiceClient client;
tf::TransformListener tf_listener_;
int markers_; 
float x_, y_;

/* 
 * Move the real block!
 */
void moveBlock( const InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM("Staging " << feedback->marker_name);     
      x_ = feedback->pose.position.x;
      y_ = feedback->pose.position.x;

      simple_arm_server::MoveArm srv;
      srv.request.pose_stampled.pose.position.x = feedback->pose.position.x;
      srv.request.pose_stampled.pose.position.y = feedback->pose.position.y;
      srv.request.pose_stampled.pose.position.z = 0.04;
      client.call(srv);
      break;
 
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM("Now moving " << feedback->marker_name); 

      simple_arm_server::MoveArm srv;
      srv.request.pose_stampled.pose.position.x = feedback->pose.position.x;
      srv.request.pose_stampled.pose.position.y = feedback->pose.position.y;
      srv.request.pose_stampled.pose.position.z = 0.04;
      client.call(srv);
      break;
  }
  
  server->applyChanges(); 
}

/* 
 * Make a box
 */
Marker makebox( InteractiveMarker &msg, float r, float g, float b )
{
  Marker m;

  m.type = Marker::CUBE;
  m.scale.x = msg.scale;
  m.scale.y = msg.scale;
  m.scale.z = msg.scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;

  return m;
}
 
/* 
 * Add a new block
 */
void addBlock( float x, float y, float rz, float r, float g, float b, int n)
{
  InteractiveMarker marker;
  marker.header.frame_id = "/base_link";
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0127;
  marker.pose.scale = 0.0254;
  
  std::stringstream conv;
  conv << n;
  marker.name = std::string("block") + conv.str(); 
  marker.description = "Another block";

  InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  marker.controls.push_back( control );
  
  control.markers.push_back( makeBox(marker, r, g, b) );
  control.always_visible = true;
  marker.controls.push_back( control );
  
  server->insert( marker );
  server->setCallback( marker.name, &moveBlock );
}

/* 
 * Process an incoming cloud
 */
void cloudCb ( const sensor_msgs::PointCloud2ConstPtr& msg )
{
  // convert to PCL
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*msg, cloud);
  
  // transform to base_link
  pcl::PointCloud<pcl::PointXYZ> cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
  if (!pcl_ros::transformPointCloud (std::string("base_link"), *cloud, cloud_transformed, tf_listener_))
  {
    ROS_ERROR ("Error converting to base_link");
    return;
  }

  // drop things on ground
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_transformed); 
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.01, 0.1);
  pass.filter(*cloud_filtered);

  // cluster
  pcl::KdTree<PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);
  
  std::vector<PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02):
  ec.setMinClusterSize(20);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(clusters);

  // for each cluster, see if it is a block
  for (size_t c = 0; c < clusters.size (); ++c)
  {  
    // find cluster centroid/color
    float x = 0; float y = 0; float z = 0; int r = 0; int g = 0; int b = 0;
    for (size_t i = 0; i < clusters[c].indices.size(); i++)
    {
        int j = clusters[c].indices[i];
        x += cloud_transformed.points[j].x;
        y += cloud_transformed.points[j].y;
        z += cloud_transformed.points[j].z;
        unsigned char * rgb = (unsigned char *) &(cloud_transformed.points[j].rgb);
        r += rgb[0];
        g += rgb[1];
        b += rgb[2];
    }
    x = x/clusters[c].indices.size();
    y = y/clusters[c].indices.size();
    z = z/clusters[c].indices.size();
    r = r/clusters[c].indices.size();
    g = g/clusters[c].indices.size();
    b = b/clusters[c].indices.size();

    bool new_ = true;
    // see if we have it detected
    for (int j = 0; j < markers_; j++){
      std::stringstream conv;
      conv << j;

      InteractiveMarker m;
      server->get( std::string("block") + conv.str(), m ); 

      if( (fabs(m.pose.position.x - x) < 0.012) &&
          (fabs(m.pose.position.y - y) < 0.012) )
      {
        new_ = false;
        break;
      }
    }

    if (new_){
      // else, add new block
      addBlock( x, y, 0.0, (float) r/255.0, (float) g/255.0, (float) b/255.0, markers++ );
    }
  }
}


int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_manipulation");
  ros::NodeHandle nh;
  client = nh.serviceClient<simple_arm_server::MoveArm>("simple_arm_server/move");
 
  // create marker server
  markers_ = 0;
  server.reset( new interactive_markers::InteractiveMarkerServer("block_controls","",false) );

  // subscribe to point cloud
  ros::Subscriber s = nh.subscribe("/camera/rgb/points", 1, cloudCb);

  // everything is done in cloud callback, just spin
  ros::spin();
  
  server.reset();  
}



