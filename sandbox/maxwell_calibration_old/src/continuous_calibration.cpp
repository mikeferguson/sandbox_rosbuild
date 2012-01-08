/* 
  Continuous Calibration for Maxwell
  Copyright (c) 2011 Michael Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holders nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "tf/transform_listener.h"

#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/features/normal_3d.h"

#include "pcl_ros/transforms.h"
#include "pcl_ros/point_cloud.h"

//#include "robot_state_publisher/robot_state_publisher.h"

/* this calibrates the head 
   params:
    height of pan servo
    camera pose 
    offset for pan servo
    offset for tilt servo 
*/

class ContinuousCalibration
{
  public:
    ContinuousCalibration(ros::NodeHandle & n):n_ (n)
    {
        ros::NodeHandle nh("~");
        nh.param("resolution",resolution_,0.005);
    
        /* windowing limits for ground detection and correction */
        nh.param("limit_x",limit_x_,1.0);
        nh.param("limit_y",limit_y_,1.0);
        nh.param("limit_z",limit_z_,0.5);

        /* subscribe/publish */
        cloud_sub_ = n.subscribe("/camera/rgb/points", 10, &ContinuousCalibration::cloud_cb, this); //TODO: change this to /camera/rgb/points
        cloud_pub_ = nh.advertise< PointCloud > ("output", 1);
    }

    /* 
     * Capture latest point cloud
     */
    void cloud_cb ( const sensor_msgs::PointCloud2ConstPtr& msg )
    {
        // this needs cleanup...
        sensor_msgs::PointCloud2::Ptr msg_transformed(new sensor_msgs::PointCloud2());
        sensor_msgs::PointCloud2::Ptr msg_filtered(new sensor_msgs::PointCloud2());
        NormalCloud normals;
        PointCloud downsampled;
        PointCloud filtered_x;
        PointCloud filtered_y;
        PointCloud xyzrgb;

        // transform to base frame
        ros::Time now = ros::Time::now();
        listener_.waitForTransform("/base_link", msg->header.frame_id, now, ros::Duration(3.0));
        if(!pcl_ros::transformPointCloud("/base_link", *msg, *msg_transformed, listener_))
        {
            ROS_ERROR("Can't transform cloud to /base_link");
            return;
        }

        // downsample
        ROS_INFO("downsample");
        pcl::VoxelGrid<sensor_msgs::PointCloud2> downsample;
        downsample.setInputCloud(msg_transformed);
        downsample.setLeafSize(resolution_, resolution_, resolution_);
        downsample.filter(*msg_filtered);
        pcl::fromROSMsg(*msg_filtered, downsampled);

        // filter in x,y,z directions
        ROS_INFO("filter");
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (downsampled.makeShared());
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-limit_x_, limit_x_);
        //pass.setFilterLimitsNegative (true);
        pass.filter(filtered_x);

        pass.setInputCloud (filtered_x.makeShared());
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-limit_y_, limit_y_);
        pass.filter(filtered_y);

        pass.setInputCloud (filtered_y.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-limit_z_, limit_z_);
        pass.filter(xyzrgb);

        // normal estimation
        ROS_INFO("normals");
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> norm_est;
        norm_est.setKSearch(25);
        norm_est.setSearchMethod (boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ());

        norm_est.setInputCloud (xyzrgb.makeShared());
        pcl::copyPointCloud (xyzrgb, normals);
        norm_est.compute (normals);
        
        // find consensus of normals
        if ( normals.points.size() > 0 )
        {
            float x = 0;
            float y = 0;
            float z = 0;
            for (size_t i = 0; i < normals.points.size (); ++i)
            {
                x += normals.points[i].normal_x;
                y += normals.points[i].normal_y;
                z += normals.points[i].normal_z;
            }
            x = x/normals.points.size();
            y = y/normals.points.size();
            z = z/normals.points.size();
            ROS_INFO("Avg. Normal: %f %f %f", x, y, z);   
        }

        // publish cloud?
        cloud_pub_.publish(xyzrgb.makeShared()); 
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private: 
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZRGBNormal> NormalCloud;

    ros::Publisher              cloud_pub_;
    ros::Subscriber             cloud_sub_; 
    ros::NodeHandle             n_;
    tf::TransformListener       listener_;

    /* parameters */
    double resolution_;
    double limit_x_;
    double limit_y_;
    double limit_z_;
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "continuous_calibration");
  ros::NodeHandle n;
  ContinuousCalibration calibration(n);
  ros::spin ();
  return 0;
}
