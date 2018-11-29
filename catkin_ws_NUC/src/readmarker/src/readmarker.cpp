//Reading AruCo markers
//Author : Dongha Chung
//Reference : https://docs.opencv.org/3.3.0/d5/dae/tutorial_aruco_detection.html

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <std_msgs/Int8.h>
#include "readmarker/markermsg.h"                                          // ******************** REVIEW 4 *********************** //
#include "readmarker/markerXY.h"                                           //Custom msg. See readermarker/msg/markerXY.msg
#include <math.h>

using namespace cv;
using namespace std;

vector<int> markerIds;                                                      //Vector to store the marker Ids
vector<vector<Point2f> > markerCorners, rejectedCandidates;                 //markerCorners : Vecter to store the coordinates of marker corners
vector<Point2f> innerMarkerCorners;                                         //Vector to store the coordinates of inner marker corners
vector<Point2f> outImageCorners;                                            //Vector to store the coordinates of output image corners,
//                                                                             //((0,0), (255,0), (255,255), (0,255) for a 256x256 size image)

Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250); //Load AruCo dictionary

Mat image_1;                                                                //Matrix where the animal image will be wraped for marker 1,2,3,4
Mat image_2;                                                                //Matrix where the animal image will be wraped for marker 5,6,7,8

int b_image1;                                                               //Integer acting as boolean to determine whether image 1 is available or not
int b_image2;                                                               //Integer acting as boolean to determine whether image 2 is available or not

ros::Publisher input_status_pub;                                            //Publisher to publish the message
ros::Publisher cmd_target_pub;

readmarker::markermsg cropped_img_msg;                                      //Message to be published containing the images, and booleans
readmarker::markerXY cmd_target;                                            //Message to save the detected marker infomation

void Initialization()                                                       //Initialization function
{
  // Corner points of 1st 2nd 3rd 4th of 256X256 square
  //   (0,0)1┌────┐2(255,0)
  //         │    │
  // (0,255)4└────┘3(255,255)
  outImageCorners.resize(4);                                                //Resize the vector into size of 4
  outImageCorners[0] = cvPoint(0.0, 0.0);                                   //First vector element : (0,0)
  outImageCorners[1] = cvPoint(255.0, 0.0);                                 //Second vector element : (255,0)
  outImageCorners[2] = cvPoint(255.0, 255.0);                               //Third vector element : (255,255)
  outImageCorners[3] = cvPoint(0.0, 255.0);                                 //Forth vector element : (0,255)
  b_image1 = 1;                                                             //Initial setting : Image 1 is available
  b_image2 = 1;                                                             //Initial setting : Image 2 is available
}

void find_marker(Mat frame)                   //Function which will run when the message "msg" from webcam node is subscribed
{
  b_image1=1;                                                               //Let image 1 is available
  b_image2=1;                                                               //Let image 1 is available

  cmd_target.width = frame.cols;                                    //Store Image's width, height in pixels
  cmd_target.height = frame.rows;

  cmd_target.marker1X[0] = NAN; cmd_target.marker1Y[0] = NAN;               // ******************** REVIEW 5 *********************** //
  cmd_target.marker1X[1] = NAN; cmd_target.marker1Y[1] = NAN;
  cmd_target.marker1X[2] = NAN; cmd_target.marker1Y[2] = NAN;
  cmd_target.marker1X[3] = NAN; cmd_target.marker1Y[3] = NAN;

  cmd_target.marker2X[0] = NAN; cmd_target.marker2Y[0] = NAN;
  cmd_target.marker2X[1] = NAN; cmd_target.marker2Y[1] = NAN;
  cmd_target.marker2X[2] = NAN; cmd_target.marker2Y[2] = NAN;
  cmd_target.marker2X[3] = NAN; cmd_target.marker2Y[3] = NAN;
  
  aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);   // ******************** REVIEW 6 *********************** //

  Mat outputImage;                                                          //Matrix where the detected markers will be shown in frame
  frame.copyTo(outputImage);                                           //Copy the frame to outputImage. The markers will be drawn afterwards
  image_1 = Mat(224,224, CV_8UC3, Scalar(255,255,255));                     //initialize image_1 to a white 256x256 image
  image_2 = Mat(224,224, CV_8UC3, Scalar(255,255,255));                     //initialize image_1 to a white 256x256 image

  if (markerIds.size() > 0)                                                 //If there is any marker detected
  {
      aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);    //Draw the markers on the outputImage

      vector<int> indice;                                                   //Vector to store the location of markerIds in consecutive manner
                                                                            //i.e if markerIds = [3 2 4 1], then indices = [4 2 1 3]
      indice.resize(8);                                                     //Resize the vector into size of 8 (since the maximum number of markers is 8)

      for(int i=0;i<4;i++)                                                  //Process to find markers 1 through 4
      {
        if(find(markerIds.begin(), markerIds.end(), i+1) != markerIds.end())  //If there exist marker with index i+1
        {
          indice[i]= distance(markerIds.begin(), find(markerIds.begin(), markerIds.end(), i+1)); //indice[i] = location of element 'i+1' in vector markerIds
        }
        else                                                                //If there are missing markers from 1 through 4, the image is not ready
          b_image1 = b_image1*0;                                            //Image 1 is not ready
      }
      for(int i=4;i<8;i++)                                                  //Process to find markers 5 through 8
      {
        if(find(markerIds.begin(), markerIds.end(), i+1) != markerIds.end())  //If there exist marker with index i+1
        {
          indice[i]= distance(markerIds.begin(), find(markerIds.begin(), markerIds.end(), i+1));  //indice[i] = location of element 'i+1' in vector markerIds
        }
        else                                                                //If there are missing markers from 5 through 8, the image is not ready
          b_image2 = b_image2*0;                                            //Image 2 is not ready
      }

      if(b_image1==1)                                                       //If image 1 is ready
      {
        //Configuration : point number
        //2┌────┐3
        // │    │
        //1└────┘0
        innerMarkerCorners.resize(4);                                       //resize the innerMarkerCorners into size of 4
        innerMarkerCorners[0] = markerCorners[indice[0]][2];                //Assign the corresponding inner marker corners of image 1
        innerMarkerCorners[1] = markerCorners[indice[1]][3];
        innerMarkerCorners[2] = markerCorners[indice[2]][0];
        innerMarkerCorners[3] = markerCorners[indice[3]][1];

                                                                            // ******************** REVIEW 7 *********************** //
        cmd_target.marker1X[0] = innerMarkerCorners[0].x; cmd_target.marker1Y[0] = innerMarkerCorners[0].y;
        cmd_target.marker1X[1] = innerMarkerCorners[1].x; cmd_target.marker1Y[1] = innerMarkerCorners[1].y;
        cmd_target.marker1X[2] = innerMarkerCorners[2].x; cmd_target.marker1Y[2] = innerMarkerCorners[2].y;
        cmd_target.marker1X[3] = innerMarkerCorners[3].x; cmd_target.marker1Y[3] = innerMarkerCorners[3].y;

        Mat H1 = findHomography(innerMarkerCorners,outImageCorners,0);      //Find the transformation matrix (homography)
                                                                            //that transformate the innerMarkerCorners coordinates to outImageCorners coordinates
        warpPerspective(frame, image_1, H1, Size(255, 255));                //Warp the frame to image_1 using the homography
                                                                            //The rest of the image will be cut out
      }
      if(b_image2==1)
      {
        innerMarkerCorners.resize(4);                                       //resize the innerMarkerCorners into size of 4 (acts as initialization of the vector)
        innerMarkerCorners[0] = markerCorners[indice[4]][2];                //Assign the corresponding inner marker corners of image 2
        innerMarkerCorners[1] = markerCorners[indice[5]][3];
        innerMarkerCorners[2] = markerCorners[indice[6]][0];
        innerMarkerCorners[3] = markerCorners[indice[7]][1];

                                                                            // ******************** REVIEW 7 *********************** //
        cmd_target.marker2X[0] = innerMarkerCorners[0].x; cmd_target.marker2Y[0] = innerMarkerCorners[0].y;
        cmd_target.marker2X[1] = innerMarkerCorners[1].x; cmd_target.marker2Y[1] = innerMarkerCorners[1].y;
        cmd_target.marker2X[2] = innerMarkerCorners[2].x; cmd_target.marker2Y[2] = innerMarkerCorners[2].y;
        cmd_target.marker2X[3] = innerMarkerCorners[3].x; cmd_target.marker2Y[3] = innerMarkerCorners[3].y;

        Mat H2 = findHomography(innerMarkerCorners,outImageCorners,0);        //Find the transformation matrix (homography)
                                                                              //that transformate the innerMarkerCorners coordinates to outImageCorners coordinates
        warpPerspective(frame, image_2, H2, Size(255, 255));                  //Warp the frame to image_2 using the homography
                                                                              //The rest of the image will be cut out
      }
  }
  else                                                                        //If there is no marker detected
  {
    b_image1 = 0;                                                             //image_1 is unavailable
    b_image2 = 0;                                                             //image_2 is unavailable
  }

  cropped_img_msg.image1_available = b_image1;                                  //Set the int message image1_available as b_image1
  cropped_img_msg.image2_available = b_image2;                                  //Set the int message image2_available as b_image2

  imencode(".jpg", image_1, cropped_img_msg.cimage1.data);                      //Set the compressed image message cimage1 as image_1
  imencode(".jpg", image_2, cropped_img_msg.cimage2.data);                      //Set the compressed image message cimage2 as image_2
  input_status_pub.publish(cropped_img_msg);                                    //Publish the message containing cropped images etc.
  cmd_target_pub.publish(cmd_target);                                           //Publish the detected marker information etc.

  Mat showImg = Mat::zeros(Size(256,512),CV_8UC3);                              //Matrix to contain images to show the wrapped images
  image_1.copyTo(showImg(Rect(0,0,image_1.cols, image_1.rows)));                //Copy image_1 to upper side of showImg
  image_2.copyTo(showImg(Rect(0,image_1.rows,image_2.cols,image_2.rows)));      //Copy image_1 to lower side of showImg

  imshow("image", showImg);                                                     //Show showImg in the window named "image"
  imshow("out", outputImage);                                                   //Show outputImage in the window named "out"

  moveWindow("image", outputImage.cols+50,20);                                  //Set the window location as (width of outputImage+50, 20)
  moveWindow("out", 50,20);                                                     //Set the window location as (width of outputImage+50, 20)
  cvWaitKey(1);                                                                 //Wait 1ms
}

int main(int argc, char** argv)
{
  Initialization();                                                             //Initialize the parameters
  ros::init(argc, argv, "marker_reader");                                       //Initialize the node "marker_reader"
  ros::NodeHandle nh;                                                           //Set the node handle as nh
  input_status_pub = nh.advertise<readmarker::markermsg>("cropped_img", 100);   //Declare publisher using the message file 'marekermsg'
  cmd_target_pub = nh.advertise<readmarker::markerXY>("markerXY", 100);         //Declare publisher using the message file 'marekermsg'


  VideoCapture cap = cv::VideoCapture(0);
  Mat frame;

  ROS_INFO("Webcam Running");
  while (nh.ok()) {
    cap >> frame;
    find_marker(frame);




    ros::Duration(0.07).sleep();
    ros::spinOnce();
  }
}
