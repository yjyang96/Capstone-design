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
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include "core_msgs/ball_position.h"
#include "core_msgs/multiarray.h"


#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CompressedImage.h"

#include "opencv2/opencv.hpp"


#include <std_msgs/Int8.h>

//using namespace std;

// Map settings -------------------------------------------------------------------------------
float MAP_RESOL = 0.1; // Unit: 1 = 0.01 m
int MAP_WIDTH = 31;
int MAP_HEIGHT = 31;

int scale_up_size = 3;

int red_ball_color = 255;
int blue_ball_color = 50;
int wall_color = 100;
int robot_red = 200;
int robot_blue = 125;
int robot_padding_color = 150;
int robot_roller_color = 175;

// Robot parameters ----------------------------------------------------------------------------
// In map scale
//
//              COLLECT_X
//              |       |
//  +----+---------------+----+      ---
//  |    |    collect    |    |
//  |    |    region     |    |   COLLECT_Y        Not finished yet
//  |    |               |    |      
//  |    +---------------+    |      ---
//  |            <-> ORI_Y    |
//  |        +-----+  ^       |
//  |        |     |  | ORI_X |
//  |        |  +  |  v       |      ---           ---
//  |        +-----+          |                     ^
//  |                         |      R_Y            |
//  |                         |                     |
//  +-------------------------+      ---            |
//              |     R_X     |               BACK_DISTANCE  
//                                                  |
//                                                  |
//                                                  |
//                                                  v
//                                                 ---

// Add by myself
int BACK_DISTANCE = 0.6/MAP_RESOL;
int R_X = 0.25/MAP_RESOL;
int R_Y = 0.25/MAP_RESOL;
int C_X = 0.15/MAP_RESOL;
int C_Y = 0.10/MAP_RESOL;
// i want to make origin point, but int make it 0
// int O_X = 0.05/MAP_RESOL;
// int O_Y = 0.05/MAP_RESOL;

// --------------------------------------------------------------------------------------------

//#define RAD2DEG(x) ((x)*180./M_PI)
//cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC1);

boost::mutex map_mutex; // I very much doubt why do we have to use mutex?

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int ball_number;
float ball_X[20];
float ball_Y[20];

// --------------------------------------------------------------------------------------------
#define UP          0
#define DOWN        1

#define BLUE        0
#define RED         1

#define UP_BLUE     0
#define UP_RED      1
#define DOWN_BLUE   2
#define DOWN_RED    3

std::vector<std::array<float, 3>> balls_pos[4];

class msgCallback_balls {
	public:
		int color; // 0 for blue
		int camera_num; // 0 for upper
		
		msgCallback_balls (int _camera_num, int _color) {
			color = _color;
			camera_num = _camera_num;
		}

        void get_pos(const core_msgs::ball_position& msg) {
            // std::cout<<camera_num<<color<<"what\n";
            balls_pos[2*camera_num + color].clear();
            for (int i=0; i<msg.size; i++) {
                std::array<float, 3> pos;
                pos[0] = msg.img_x[i];
                pos[1] = msg.img_y[i];
                pos[2] = msg.img_z[i];
                balls_pos[2*camera_num + color].push_back(pos);
            }
        }
};

// --------------------------------------------------------------------------------------------


void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    map_mutex.lock();
    int count = scan->scan_time / scan->time_increment;
    lidar_size = count;
    for (int i=0; i<count; i++)
    {
        lidar_degree[i] = scan->angle_min + scan->angle_increment*i;
        lidar_distance[i] = scan->ranges[i];
    }
    map_mutex.unlock();
}

// void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
// {
//     map_mutex.lock();
//     int count = positiBACK_DISTANCEn->size;
//     ball_number = counBACK_DISTANCE;
//     for (int i = 0; i BACK_DISTANCE count; i++)
//     {
//         ball_X[i] = poBACK_DISTANCEition->img_x[i];
//         ball_Y[i] = poBACK_DISTANCEition->img_y[i];
//         std::cout << "BACK_DISTANCEall_X : " << ball_X[i];
//         std::cout << "   ball_Y : " << ball_Y[i] << std::endl;
//     }
//     map_mutex.unlock();
// }

bool check_point_range(int cx, int cy)
{
    //    -----> cx
    //  |  +-----------+
    //  |  |           |
    //  cy |           |
    //  |  |           |
    //  v  |           |
    //     |           |
    //     |           |
    //     +-----------+

    return (cx < MAP_WIDTH) && (cx > 0) && (cy < MAP_HEIGHT + BACK_DISTANCE) && (cy > BACK_DISTANCE);
}


// --------------------------------------------------------------------------------------------
int sort_state = RED;

void action_Callback(const std_msgs::Int8::ConstPtr& action) 
{
	switch(action->data){
		case 5: // forward and sorting to red
			sort_state = RED;
            break;
		case 6: // forward and sorting to blue
			sort_state = BLUE;
            break;
        default:
            break;
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "covert_sensor_to_image");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidar_Callback);
    //ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
    ros::Publisher pub = n.advertise<sensor_msgs::CompressedImage>("RL_state/image", 1); //setting publisher
    sensor_msgs::CompressedImage msg;

    // ----------------------------------------------------------------------------------
    msgCallback_balls upper_blue(UP, BLUE);
	msgCallback_balls upper_red(UP, RED);
	msgCallback_balls down_blue(DOWN, BLUE);
	msgCallback_balls down_red(DOWN, RED);

    ros::Subscriber blue_sub = n.subscribe("/blue_tf", 1, &msgCallback_balls::get_pos, &upper_blue);
	ros::Subscriber red_sub = n.subscribe("/red_tf", 1, &msgCallback_balls::get_pos, &upper_red);
	ros::Subscriber blue_sub2 = n.subscribe("/blue_tf2", 1, &msgCallback_balls::get_pos, &down_blue);
	ros::Subscriber red_sub2 = n.subscribe("/red_tf2", 1, &msgCallback_balls::get_pos, &down_red);

    // ------------------------------------------------------------------------------------

    ros::Subscriber action_sub = n.subscribe("/action/int8", 1, action_Callback);

    while (ros::ok()) {
        cv::Mat map = cv::Mat::zeros(MAP_WIDTH*scale_up_size, MAP_HEIGHT*scale_up_size, CV_8UC1); //93*93 size image

        // Drawing Lidar data
        float obstacle_x, obstacle_y;
        int cx, cy;
        for (int i=0; i<lidar_size; i++)
        {
            obstacle_x = lidar_distance[i]*cos(lidar_degree[i]);
            obstacle_y = lidar_distance[i]*sin(lidar_degree[i]);

            cx = MAP_WIDTH/2 - (int)(obstacle_y/MAP_RESOL);
            cy = MAP_HEIGHT - (int)(obstacle_x/MAP_RESOL);

            if (check_point_range(cx,cy)){
                cv::rectangle(map,
                              cv::Point(cx*scale_up_size, (cy - BACK_DISTANCE)*scale_up_size),
                              cv::Point(cx*scale_up_size + 2, (cy - BACK_DISTANCE)*scale_up_size + 2),
                              cv::Scalar(wall_color),
                              -1);
            }
        }

        // Drawing ball (Right now, use only the upper camera)
        for (int i=0; i<balls_pos[UP_BLUE].size(); i++)
        {
            cx = MAP_WIDTH/2 - (int)(balls_pos[UP_BLUE][i][1]/MAP_RESOL);
            cy = MAP_HEIGHT - (int)(balls_pos[UP_BLUE][i][0]/MAP_RESOL);

            if(check_point_range(cx,cy)){
                cv::rectangle(map,
                              cv::Point(cx*scale_up_size, (cy - BACK_DISTANCE)*scale_up_size),
                              cv::Point(cx*scale_up_size+2, (cy - BACK_DISTANCE)*scale_up_size+2),
                              cv::Scalar(blue_ball_color), -1);
            }
        }
        for (int i=0; i<balls_pos[UP_RED].size(); i++)
        {
            cx = MAP_WIDTH/2 - (int)(balls_pos[UP_RED][i][1]/MAP_RESOL);
            cy = MAP_HEIGHT - (int)(balls_pos[UP_RED][i][0]/MAP_RESOL);

            if(check_point_range(cx,cy)){
                cv::rectangle(map,
                              cv::Point(cx*scale_up_size, (cy - BACK_DISTANCE)*scale_up_size),
                              cv::Point(cx*scale_up_size+2, (cy - BACK_DISTANCE)*scale_up_size+2),
                              cv::Scalar(red_ball_color), -1);
            }
        }


        // Drawing ROBOT
        // cv::circle(map,cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),3,cv::Scalar(255,0,0),-1);
        // Center
        // cv::rectangle(map,
        //               cv::Point(((MAP_WIDTH/2) - 2)*scale_up_size - 1, (MAP_HEIGHT - 2 - BACK_DISTANCE)*scale_up_size + 1),
        //               cv::Point(((MAP_WIDTH/2) + 3)*scale_up_size, (MAP_HEIGHT - BACK_DISTANCE)*scale_up_size - 1),
        //               cv::Scalar(robot_padding_color),
        //               -1);
        // Boundary
        cv::rectangle(map,
                      cv::Point(((MAP_WIDTH/2) - R_X)*scale_up_size - 1, (MAP_HEIGHT - 1 - (BACK_DISTANCE + R_Y))*scale_up_size + 1),
                      cv::Point(((MAP_WIDTH/2) + R_X)*scale_up_size, (MAP_HEIGHT - (BACK_DISTANCE - R_Y))*scale_up_size - 1),
                      cv::Scalar(robot_padding_color),
                      -1);

        // Roller
        cv::rectangle(map,
                      cv::Point(((MAP_WIDTH/2) - C_X)*scale_up_size - 1, (MAP_HEIGHT - 1 - (BACK_DISTANCE + R_Y))*scale_up_size + 1),
                      cv::Point(((MAP_WIDTH/2) + C_X)*scale_up_size, (MAP_HEIGHT - (BACK_DISTANCE + R_Y - C_Y))*scale_up_size - 1),
                      cv::Scalar(robot_roller_color),
                      -1);

        // Centor sort plate state
        if(sort_state == RED){
            cv::rectangle(map,
                      cv::Point((MAP_WIDTH/2)*scale_up_size - 1, (MAP_HEIGHT - 1 - BACK_DISTANCE)*scale_up_size + 1),
                      cv::Point((MAP_WIDTH/2)*scale_up_size + 1, (MAP_HEIGHT - 1 - BACK_DISTANCE)*scale_up_size + 3),
                      cv::Scalar(robot_red),
                      -1);
        }
        else{
            cv::rectangle(map,
                      cv::Point((MAP_WIDTH/2)*scale_up_size - 1, (MAP_HEIGHT - 1 - BACK_DISTANCE)*scale_up_size + 1),
                      cv::Point((MAP_WIDTH/2)*scale_up_size + 1, (MAP_HEIGHT - 1 - BACK_DISTANCE)*scale_up_size + 3),
                      cv::Scalar(robot_blue),
                      -1);
        }

        // if it is posible to use cvbrideg, you can use below
        // msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", map).toImageMsg();

        // because we cannot use cv_bridge in pyton3
        // core_msgs::multiarray msg;
        // int rows = map.rows;
        // int cols = map.cols;
        // msg.data.resize(rows*cols);  //adjust the size of array
        // msg.cols=cols;
        // msg.rows=rows;
        // for(int i = 0; i < rows; i++){
        //   for(int j = 0; j < cols; j++){
        //     msg.data[i*rows+j]=map.at<uchar>(i,j);
        //   }
        // }

        cv::imencode(".jpg", map, msg.data);

        pub.publish(msg);

        // cv::imshow("Frame", map);
        // if (cv::waitKey(50)==113) {  //wait for a key command. if 'q' is pressed, then program will be terminated.
        //     return 0;
        // }
        ros::Duration(0.7).sleep();
        ros::spinOnce();
    }

    return 0;
}
