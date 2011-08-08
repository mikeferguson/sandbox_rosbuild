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
 */

/* 
 * Quick and Dirty Calibration
 * Author: Michael Ferguson
 */

#include <pthread.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> 
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

/** @brief Helper function to convert Eigen transformation to tf -- thanks to Garret Gallagher */
tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
    btMatrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
               trans(1,0),trans(1,1),trans(1,2),
               trans(2,0),trans(2,1),trans(2,2));
    btTransform ret;
    ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
}

/** @brief ... */
class QNDcalibration{
  public:
    static const int square_size = 0.05715;

    QNDcalibration(ros::NodeHandle & n): frame_id_(""), nh_ (n) 
    {
        ros::NodeHandle nh ("~");

        // subscribe to cloud, publish points to capture with arm
        cloud_sub_ = nh_.subscribe("/camera/rgb/points", 1, &QNDcalibration::cloudCallback, this);
        cloud_pub_ = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("output", 1);

        timer_ = nh_.createTimer(ros::Duration(0.1), &QNDcalibration::publishCallback, this);
    }

    void publishCallback(const ros::TimerEvent&)
    {
        if(frame_id_.compare("") != 0)
            br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), frame_id_, "chess_board"));
    }

    void cloudCallback ( const sensor_msgs::PointCloud2ConstPtr& msg )
    {
        if(frame_id_.compare("") != 0) return;

        sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);

        // convert cloud to PCL
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*msg, cloud);
 
        // get an OpenCV image from the cloud
        pcl::toROSMsg (cloud, *image_msg);

        // convert image to OpenCV 
        try
        {
            bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");
            //ROS_INFO("New image");
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("Conversion failed");
        }
        
        // find chessboard corners
        std::vector<cv::Point2f> points;
        if(!cv::findChessboardCorners( bridge_->image, cv::Size(7,7), points, 
                                      cv::CALIB_CB_ADAPTIVE_THRESH))
        {
            ROS_INFO("No board found.");
            return;
        }           

        // determine orientation        
        std::vector<int> indices;
        cv::Point2f p0_ = points[0];
        pcl::PointXYZRGB p0 = cloud((int)p0_.x, (int)p0_.y);
        //ROS_INFO_STREAM( p0.x << "," << p0.y );
        if( (p0.x < 0) && (p0.y < 0) ){
            indices.push_back(42); indices.push_back(48); indices.push_back(0); indices.push_back(6);
        }else if( (p0.x < 0) && (p0.y > 0) ){
            indices.push_back(0); indices.push_back(42); indices.push_back(6); indices.push_back(48);
        }else if( (p0.x > 0) && (p0.y < 0) ){
            indices.push_back(48); indices.push_back(6); indices.push_back(42); indices.push_back(0);
        }else if( (p0.x > 0) && (p0.y > 0) ){
            indices.push_back(6); indices.push_back(0); indices.push_back(48); indices.push_back(42);
        }

        // extract points 
        pcl::PointCloud<pcl::PointXYZ> corners;
        corners.header.frame_id  = msg->header.frame_id;
        corners.header.stamp  = msg->header.stamp;

        for( size_t i = 0; i < indices.size(); i++ )
        {
            int j = indices[i];
            cv::Point2f pt = points[j];
            pcl::PointXYZRGB p = cloud((int)pt.x, (int)pt.y);
            corners.push_back( pcl::PointXYZ(p.x, p.y, p.z) );
        }

        // estimate
        pcl::PointCloud<pcl::PointXYZ> ideal_corners;
        ideal_corners.push_back( pcl::PointXYZ(0.05715, 0.05715, 0) );      // lower left
        ideal_corners.push_back( pcl::PointXYZ(0.05715*7, 0.05715, 0) );    // lower right
        ideal_corners.push_back( pcl::PointXYZ(0.05715, 0.05715*7, 0) );    // upper left
        ideal_corners.push_back( pcl::PointXYZ(0.05715*7, 0.05715*7, 0) );  // upper right
        Eigen::Matrix4f t;
        pcl::estimateRigidTransformationSVD( corners, ideal_corners, t );

        // output cloud & board frame   
        frame_id_ = msg->header.frame_id;      
        transform_ = tfFromEigen(t.inverse());
    }
    
    void calibrate()
    {
        ros::Duration(1.0).sleep();
        pcl::PointCloud<pcl::PointXYZ> raw_capture_cloud;
        raw_capture_cloud.header.frame_id  = "chess_board";
        raw_capture_cloud.header.stamp  = ros::Time::now();
        ros::Duration(1.0).sleep();
        raw_capture_cloud.push_back( pcl::PointXYZ(0.05715*0.5, 0.05715*0.5, 0) );    // center of a1
        raw_capture_cloud.push_back( pcl::PointXYZ(0.05715*7.5, 0.05715*0.5, 0) );    // center of h1
        raw_capture_cloud.push_back( pcl::PointXYZ(0.05715*2.5, 0.05715*2.5, 0) );    // center of c3
        raw_capture_cloud.push_back( pcl::PointXYZ(0.05715*5.5, 0.05715*2.5, 0) );    // center of f3
        pcl_ros::transformPointCloud (frame_id_, raw_capture_cloud, capture_cloud, listener_);
        //listener_.transformPointCloud(/*msg->header.frame_id*/ "openni_rgb_optical_frame", capture_cloud, capture_cloud);
        cloud_pub_.publish( capture_cloud );

        Eigen::Matrix4f t;
        pcl::PointCloud<pcl::PointXYZ> captured;
        // ask user to move to points (and capture using tf)
        std::cout << "Move arm to center of a1 and press enter.";        
        std::cin.ignore();
        geometry_msgs::PointStamped pt;    // all 0
        pt.header.frame_id = "gripper_link";
        geometry_msgs::PointStamped pt_out;
        listener_.transformPoint(frame_id_, pt, pt_out);
        captured.push_back(pcl::PointXYZ(pt_out.point.x, pt_out.point.y, pt_out.point.z));

        std::cout << "Move arm to center of h1 and press enter.";        
        std::cin.ignore();
        listener_.transformPoint(frame_id_, pt, pt_out);
        captured.push_back(pcl::PointXYZ(pt_out.point.x, pt_out.point.y, pt_out.point.z));

        std::cout << "Move arm to center of c3 and press enter.";        
        std::cin.ignore();
        listener_.transformPoint(frame_id_, pt, pt_out);
        captured.push_back(pcl::PointXYZ(pt_out.point.x, pt_out.point.y, pt_out.point.z));

        std::cout << "Move arm to center of f3 and press enter.";        
        std::cin.ignore();
        listener_.transformPoint(frame_id_, pt, pt_out);
        captured.push_back(pcl::PointXYZ(pt_out.point.x, pt_out.point.y, pt_out.point.z));

        pcl::estimateRigidTransformationSVD( captured, capture_cloud, t );

        // output cloud & board frame       
        tf::Transform transform = tfFromEigen(t.inverse());

        double roll, pitch, yaw;
        transform.getBasis().getEulerYPR(yaw, pitch, roll);

        // output URDF updates
        ROS_INFO("Copy the following into your URDF:\n");
        ROS_INFO_STREAM("  <property name='calib_cam_x' value='" << t(0,3) << "' />\n" <<
                        "  <property name='calib_cam_y' value='" << t(1,3) << "' />\n" <<
                        "  <property name='calib_cam_z' value='" << t(2,3) << "' />\n" <<
                        "  <property name='calib_cam_rr' value='" << roll << "' />\n" <<
                        "  <property name='calib_cam_rp' value='" << pitch << "' />\n" <<
                        "  <property name='calib_cam_ry' value='" << yaw << "' />\n");

        ros::Duration(1.0).sleep();
    }


    std::string frame_id_;
    pcl::PointCloud<pcl::PointXYZ> capture_cloud;

  private:
    /* node handles, subscribers, publishers, etc */
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    tf::TransformListener listener_;
    tf::TransformBroadcaster br_;
    tf::Transform transform_;
    ros::Timer timer_;
    cv_bridge::CvImagePtr bridge_;
};

void* startThread(void* args)
{
    QNDcalibration* cal = (QNDcalibration*)args;
    cal->calibrate();
    return 0;
}

int main (int argc, char **argv)
{
  pthread_t cal_thread;

  ros::init(argc, argv, "qnd_calibration");
  ros::NodeHandle n;
  QNDcalibration cal(n);

  while(cal.frame_id_.compare("") == 0)
    ros::spinOnce();

  pthread_create( &cal_thread, NULL, startThread, (void*) &cal);
  //cal.calibrate();
  ros::spin();
  pthread_join( cal_thread, NULL);
 
  return 0;
}
