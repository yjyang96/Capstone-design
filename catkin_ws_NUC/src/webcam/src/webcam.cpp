#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  VideoCapture cap = cv::VideoCapture(1);                           //Capture the image from webcam 0
  Mat frame;                                                        //Matrix to store the image
  ros::init(argc, argv, "image_publisher");                         //Initialize the node 'image_publisher'
  ros::NodeHandle nh;                                               //Set the node handle as nh
  image_transport::ImageTransport it(nh);                           //Set the image transport as it using node handle nh
  image_transport::Publisher pub = it.advertise("camera/image", 1); // ******************** REVIEW 1 *********************** //
  sensor_msgs::ImagePtr msg;                                        //image message msg
  ROS_INFO("Webcam Running");
  while (nh.ok()) {
    cap >> frame;                                                   //Import the captured image into 'frame'
    imshow("Webcam",frame);                                         //Show the image into the window "Webcam"
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg(); // ******************** REVIEW 2 *********************** //
    pub.publish(msg);                                               // ******************** REVIEW 3 *********************** //
    waitKey(1);                                                     //wait 1ms
  }
}
