#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>

#include "ros/ros.h"
#include "core_msgs/markermsg.h"
#include "sensor_msgs/CompressedImage.h"


#include "opencv2/opencv.hpp"

using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "testimage_publisher");
    ros::NodeHandle n;
    ros::Publisher pub;
    // pub = n.advertise<sensor_msgs::CompressedImage>("catdog/image", 1); //setting publisher
    pub = n.advertise<core_msgs::markermsg>("cropped_img", 100);

    cv::Mat image_1;
    cv::Mat image_2;
    ///////////////////////////////////
    // Write down your image director//
    //////////////////////////////////
    image_1 = cv::imread("/home/nvidia/capstone_design_ROS/capstone_design/src/cnn_for_jetson/src/cat.325.jpg",CV_LOAD_IMAGE_COLOR);
    image_2 = cv::imread("/home/nvidia/capstone_design_ROS/capstone_design/src/cnn_for_jetson/src/dog.232.jpg",CV_LOAD_IMAGE_COLOR);

    while(ros::ok){

      core_msgs::markermsg msg;
      msg.image1_available = 1;
      msg.image2_available = 1;
      cv::imencode(".jpg", image_1, msg.cimage1.data);
      cv::imencode(".jpg", image_2, msg.cimage2.data);
      pub.publish(msg);

      cv::imshow("Frame1",image_1);
      cv::imshow("Frame2",image_2);

      if(cv::waitKey(30)==113){  //wait for a key command. if 'q' is pressed, then program will be terminated.
        return 0;
      };

      ros::spinOnce();
    }

    return 0;
}
