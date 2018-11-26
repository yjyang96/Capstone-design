//
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include "core_msgs/ball_position.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#define BLUE 0
#define RED 1
// ?
using namespace std;
using namespace cv;


// high and low range for red
int low_h2_r=160, high_h2_r=180;
int low_h_r=0, low_s_r=100, low_v_r=136;
int high_h_r=10, high_s_r=255, high_v_r=255;

// high and low for blue
int low_h_b=110, low_s_b=100, low_v_b=100;
int high_h_b=130, high_s_b=255, high_v_b=255;

// function declaration for intToString and floatToString
string intToString(int n);
string floatToString(float f);

// declare morphological operations
void morphOps(Mat &thresh);

// declare pixel2point function
vector<float> pixel2point(Point center, int radius);

//declare ball detect positions

vector<vector<float> > getBallPosition( vector<float> radius, vector<Point2f> center, Scalar color, int &ball_num, Mat &result);
// declare center outputs
void remove_duplicate(vector<Point2f> &centers, vector<float> &radii);
void sendBall(int ball_num, vector<vector<float> > ball_position, ros::Publisher markers, int color);

// canny parameters for red
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;

// canny parameters for blue
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

// ball diameter
float fball_diameter = 0.074 ; // meter

// camera calibration parameters
Mat distCoeffs;
float intrinsic_data[9] = {636.792913,  0, 329.63907, 0, 635.580978, 238.191252, 0, 0, 1};
float distortion_data[5] = {0.028996, -0.136993, 0.002800, 0.000258, 0.00000};

// font and size for text on screen
double fontScale = 2;
int thickness = 3;
String text ;

// ?
int iMin_tracking_ball_size = 0;

// ?
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_detect_node_1"); //init ros nodd
    ros::NodeHandle nh; //create node handler
    ros::Publisher blue_markers = nh.advertise<visualization_msgs::Marker>("/blue_markers",1);
    ros::Publisher red_markers = nh.advertise<visualization_msgs::Marker>("/red_markers",1);

    // ?
    Mat frame, bgr_frame, hsv_frame,
        hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue,
        hsv_frame_red_blur, hsv_frame_blue_blur,
        hsv_frame_red_canny, hsv_frame_blue_canny, result;

    // ?
    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    // ?
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);

    // contour heirarchy vectors
    vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;

    // contour vectors
    vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;

    // webcam on
    VideoCapture cap(0);

    // display window
    // namedWindow("Video Capture", WINDOW_NORMAL);
    // namedWindow("Object Detection_HSV_Red", WINDOW_NORMAL);
    // namedWindow("Object Detection_HSV_Blue", WINDOW_NORMAL);
    // namedWindow("Canny Edge for Red Ball", WINDOW_NORMAL);
    // namedWindow("Canny Edge for Blue Ball", WINDOW_NORMAL);
    // namedWindow("Result", WINDOW_NORMAL);

    // display window location
    // moveWindow("Video Capture", 50,  0);
    // moveWindow("Object Detection_HSV_Red",  50,370);
    // moveWindow("Object Detection_HSV_Blue",470,370);
    // moveWindow("Canny Edge for Red Ball",   50,730);
    // moveWindow("Canny Edge for Blue Ball", 500,730);
    // moveWindow("Result", 470, 0);

    // ofstream myfile("/home/capstone5/example.txt", std::ios::app);
    // ros::Time start = ros::Time::now();

    // ?
    while (ros::ok()) {
        waitKey(1);
        // ?
        cap>>frame;

        // ?
        if (frame.empty())
            break;

        // ?
        undistort(frame, calibrated_frame, intrinsic, distCoeffs);

        // ?
        result = calibrated_frame.clone();

        // ?
        medianBlur(calibrated_frame, calibrated_frame, 3);

        // ?
        cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

        // threshold hsv values of red, blue and green from the hsv_frame
        inRange(hsv_frame,Scalar(0,150,0),Scalar(10,255,255),hsv_frame_red1);
        inRange(hsv_frame,Scalar(165,150,0),Scalar(185,255,255),hsv_frame_red2);
        inRange(hsv_frame,Scalar(100,150,0),Scalar(130,255,255),hsv_frame_blue);

        // weighted sum of low range and high range red
        addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

        // apply morphological operations on thresholded images
        morphOps(hsv_frame_red);
        morphOps(hsv_frame_blue);

        // blur the images
        GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
        GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);

        // canny
        Canny(hsv_frame_red_blur, hsv_frame_red_canny, lowThreshold_r, lowThreshold_r*ratio_r, kernel_size_r);
        Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);

        // find contours
        findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
        findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

        vector<vector<Point> > contours_r_poly( contours_r.size() );
        vector<vector<Point> > contours_b_poly( contours_b.size() );

        vector<Point2f>center_r( contours_r.size() );
        vector<Point2f>center_b( contours_b.size() );

        vector<float>radius_r( contours_r.size() );
        vector<float>radius_b( contours_b.size() );

        // approximate polynomial and find enclosing cirlce for red
        for( size_t i = 0; i < contours_r.size(); i++ ){
            // ?
            approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
            // ?

            minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
            if (radius_r[i] > iMin_tracking_ball_size){
                float pixel_correction = 12 - (94.156/radius_r[i]);
                radius_r[i] = radius_r[i] - roundf(pixel_correction);
            }
                    //cout<<radius_r[i]<<endl;
        }

        // approximate polynomial and find enclosing cirlce for blue
        for( size_t i = 0; i < contours_b.size(); i++ ){
            // ?
            approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
            // ?
            minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
            if (radius_b[i] > iMin_tracking_ball_size){
                float pixel_correction = 12 - (120.156/radius_b[i]);
                radius_b[i] = radius_b[i] - roundf(pixel_correction);
            }
        }

        ///remove reflections
        remove_duplicate(center_r, radius_r);
        remove_duplicate(center_b, radius_b);

        // cout<<radius_g.size()<< "     " <<contours_g.size()<<endl;

        int ball_num = 0; Scalar color_r, color_b; color_r  = Scalar(0,0,255); color_b = Scalar(255,0,0);
        vector<vector<float> > ball_position_r = getBallPosition(radius_r, center_r, color_r, ball_num, result);
        // sendBall(ball_num, ball_position_r, red_pub, red_markers, RED);
        sendBall(ball_num, ball_position_r, red_markers, RED);
        vector<vector<float> > ball_position_b = getBallPosition(radius_b, center_b, color_b, ball_num, result);
        // sendBall(ball_num, ball_position_b, blue_pub, blue_markers, BLUE);
        sendBall(ball_num, ball_position_b, blue_markers, BLUE);

        // imshow("Video Capture",calibrated_frame);
        // imshow("Object Detection_HSV_Red",hsv_frame_red);
        // imshow("Object Detection_HSV_Blue",hsv_frame_blue);
        // imshow("Canny Edge for Red Ball", hsv_frame_red_canny);
        // imshow("Canny Edge for Blue Ball", hsv_frame_blue_canny);
        // imshow("Result", result);

        ros::spinOnce();

    }

    // myfile.close();
    return 0;
}

// ?
string intToString(int n)
{
    stringstream s;
    s << n;
    return s.str();
}

// ?
string floatToString(float f)
{
    ostringstream buffer;
    buffer << f;
    return buffer.str();
}

// apply morphological operations
void morphOps(Mat &thresh){

    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle which is used as a kernel in erosion and dilation
    //maybe increase the erosion structuring element for removing large sized reflections
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(10,10));

    // ?
    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);

    // ?
    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}

// ?
vector<float> pixel2point(Point center, int radius){

    // ?
    vector<float> position;
    float x, y, u, v, Xc, Yc, Zc;

    // ?
    x = center.x;
    y = center.y;

    // ?
    u = (x-intrinsic_data[2])/intrinsic_data[0];
    v = (y-intrinsic_data[5])/intrinsic_data[4];

    // ?
    Zc = (636.2*fball_diameter)/(2*(float)radius) + fball_diameter/2 - 0.016;
    //dist = (s*72*636.2/d);
    Xc = u*Zc ;
    Yc = v*Zc ;
    //cout<<Zc<<endl;
    // ?
    // Xc = roundf(Xc * 1000) / 1000;
    // Yc = roundf(Yc * 1000) / 1000;
    // Zc = roundf(Zc * 1000) / 1000;
    // cout<<Xc<<"    "<<Yc<<"        "<<Zc<<endl;
    // ?
    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(Zc);
    // cout<<position.size()<<endl;
    return position;
}

///get ball positions

vector<vector<float> > getBallPosition( vector<float> radius, vector<Point2f> center, Scalar color, int &ball_num, Mat &result){
    ball_num = 0;
    vector<vector<float> > ball_positions;
    for( size_t i = 0; i< center.size(); i++ ){
    // ?
        if (radius[i] > iMin_tracking_ball_size){
            //drawContours( hsv_frame_canny, contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            vector<float> ball_position;
            // ?
            ball_position = pixel2point(center[i], radius[i]);
            ball_positions.push_back(ball_position);
            // float isx = ball_position[0];
            // float isy = ball_position[1];
            // float isz = ball_position[2];
            // string sx = floatToString(isx);
            // string sy = floatToString(isy);
            // string sz = floatToString(isz);
            // text = "ball:" + sx + "," + sy + "," + sz;
            // // ?
            // putText(result, text, center[i],2,1,Scalar(0,255,0),2);
            // ?
            circle( result, center[i], (int)radius[i], color, 2, 8, 0 );
            ball_num++;
            //cout<<radius_r[i]<<endl;
        }
    }
    return ball_positions;
}


///remove reflections functions

void remove_duplicate(vector<Point2f> &centers, vector<float> &radii){
    size_t i = 0;
    while (i < radii.size()){
        bool something = true;
        for (size_t j = 0; j < radii.size() ; j++){
            if (i!=j&&(norm(centers[i] - centers[j]) < radii[j])){
                centers.erase(centers.begin()+i);
                radii.erase(radii.begin()+i);
                something = false;
                break;
            }
        }
        if(something){
            i++;
        }
    }
}

// void sendBall(int ball_num, vector<vector<float> > ball_position, ros::Publisher pub, ros::Publisher markers, int color){
void sendBall(int ball_num, vector<vector<float> > ball_position, ros::Publisher markers, int color){
    // core_msgs::ball_position msg;
    // msg.size=ball_num;

    visualization_msgs::Marker ball_list;  //declare marker
    std_msgs::ColorRGBA c;
    switch(color){
        case RED:
            ball_list.ns = "red_balls";   //name of markers
            ball_list.id = RED; //set the marker id. if you use another markers, then make them use their own unique ids
            c.r = 1.0;  //set the color of the balls. You can set it respectively. 
            c.g = 0;
            c.b = 0;
            c.a = 1.0;
            break;
        case BLUE:
            ball_list.ns = "blue_balls";   //name of markers
            ball_list.id = BLUE; //set the marker id. if you use another markers, then make them use their own unique ids
            c.r = 0;  //set the color of the balls. You can set it respectively. 
            c.g = 0;
            c.b = 1.0;
            c.a = 1.0;
            break;
        default:
            cout<<"wrong color input\n";
    }

    ball_list.header.frame_id = "/camera_link1";  //set the frame
    ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
    ball_list.action = visualization_msgs::Marker::ADD;  
    ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw) 
    ball_list.pose.position.y=0;
    ball_list.pose.position.z=0;
    ball_list.pose.orientation.x=0;
    ball_list.pose.orientation.y=0;
    ball_list.pose.orientation.z=0;
    ball_list.pose.orientation.w=1.0;

    ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker

    ball_list.scale.x=0.10; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
    ball_list.scale.y=0.10;
    ball_list.scale.z=0.10;

    if (ball_num) {
        // msg.img_x.resize(ball_num);
        // msg.img_y.resize(ball_num);
        // msg.img_z.resize(ball_num);
        // for(int i=0; i<ball_num; i++){
        //     msg.img_x[i]=ball_position[i][0];
        //     msg.img_y[i]=ball_position[i][1];
        //     msg.img_z[i]=ball_position[i][2];
        // }

        for(int k=0;k<ball_num;k++){
            geometry_msgs::Point p;
            p.x = ball_position[k][0];   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
            p.y = ball_position[k][1];
            p.z = ball_position[k][2];
            ball_list.points.push_back(p);
            ball_list.colors.push_back(c);
        }
    }
    // pub.publish(msg);
    markers.publish(ball_list);
}

