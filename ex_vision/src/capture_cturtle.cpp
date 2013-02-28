/* 
 * Example code of how to convert ROS images to OpenCV's IplImage
 * This is the solution to HW2, for cturtle release systems.
 * 
 * See also cv_bridge tutorials: 
 *   http://www.ros.org/wiki/cv_bridge
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

/* 
 * This is a callback which recieves images and processes them. 
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // convert image into openCV format
  sensor_msgs::CvBridge bridge;
  IplImage * img = NULL;
  try {
    // bgr8 is the pixel encoding -- 8 bits per color, organized as blue/green/red
    img = bridge.imgMsgToCv(msg, "bgr8");
  } catch (sensor_msgs::CvBridgeException& e) {
    // all print statements should use a ROS_ form, don't cout!
    ROS_ERROR("Conversion failed");
  }

  // we could do anything we want with the image here
  // for now, we'll blur using a median blur
  cvSmooth(img, img, CV_MEDIAN, 31);  

  // show the image
  cvShowImage("image_view",img);

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
  cvNamedWindow("image_view");
  cvStartWindowThread();

  // part 2.1 of hw2 -- subscribe to a topic called image
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);

  // this node is callback based -- just spin()
  ros::spin();

  // done, clean up
  cvDestroyWindow("image_view");

  return 0;
}

