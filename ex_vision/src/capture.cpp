/* 
 * Example code of how to convert ROS images to OpenCV's cv::Mat
 * This is the solution to HW2, for diamondback release systems.
 * 
 * See also cv_bridge tutorials: 
 *   http://www.ros.org/wiki/cv_bridge
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

/* 
 * This is a callback which recieves images and processes them. 
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // convert image into openCV formate
  cv_bridge::CvImagePtr bridge;
  try
  {
    // bgr8 is the pixel encoding -- 8 bits per color, organized as blue/green/red
    bridge = cv_bridge::toCvCopy(msg, "bgr8");
  }
  catch(cv_bridge::Exception& e)
  {
    // all print statements should use a ROS_ form, don't cout!
    ROS_ERROR("Conversion failed");
  }

  // we could do anything we want with the image here
  // for now, we'll blur using a median blur
  cv::medianBlur(bridge->image, bridge->image, 31);

  // show the image
  cv::imshow("image_view", bridge->image);
  cv::waitKey(3);

  // topic callbacks do not return -- they are always void!
}

/* 
 * We need a main function in all nodes 
 */
int main(int argc, char** argv)
{
  // initialize a node called hw2
  ros::init(argc, argv, "hw2");
  ros::NodeHandle nh;
    
  // create a window to display results in
  cv::namedWindow("image_view");

  // part 2.1 of hw2 -- subscribe to a topic called image
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);

  // this node is callback based -- just spin()
  ros::spin();

  return 0;
}

