//Making AruCo markers
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

using namespace cv;
using namespace std;

int main()
{
  Mat markerImage;  //Matrix which the markers will be created in
  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);  //Load the AruCo marker dictionary
  string imgtitle_dir = "/home/dongha/Capstone_Programs/src/webcam/src/";   //directory where the images will be saved
  string imgtitle_pre = "Marker No. ";  //Image title prefix
  string img_format = ".jpg";   //Image format
  string imgtitle;  //String which will contain the whole directory and image title
  for(int i=1;i<9;i++)
  {
    aruco::drawMarker(dictionary, i, 200, markerImage, 1);  //Draw the Marker ith marker as image size 200x200 pixels to matrix 'markerImage' with border or 1  pixel
    imgtitle = imgtitle_dir+imgtitle_pre+boost::lexical_cast<std::string>(i)+img_format;    //Make imgtitle as a combination of image directory, image title prefix, image number, and image format
    imwrite(imgtitle,markerImage); //Save the image as 'imagetitle'
  }
  return 0;
}
