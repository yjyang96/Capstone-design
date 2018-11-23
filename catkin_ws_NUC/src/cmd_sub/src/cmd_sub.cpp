#include <stdio.h>
#include <stdlib.h>
#include <cmath>                                                                //For calculation something
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>                                                          //For TCP/IP connection
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>                                                 //For msg of the topic "/cmd_vel"
#include "std_msgs/Int8.h"                                                       //For msg of the topic "/action/int8"

int port;                                                                        //Port number for TCP/IP
std::string ip_address;                                                          //Address for TCP/IP

double max_linear_vel;                                                           //Maximaum linear/angular velocity
double max_angular_vel;                                                          //Match thease with myRIO's corresponding parmameters

int c_socket;
struct sockaddr_in c_addr;
float data[24];
ros::Time Last_dqn;                                                               //Last time when the topic for DQN is recieved
bool Dqn_start;                                                                   //True if DQN is running

void dataInit()
{
    data[0] = 0; //lx*data[3];
    data[1] = 0; //ly*data[3];
    data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
    data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
    data[4] = 0; //rx*data[7];
    data[5] = 0; //ry*data[7];
    data[6] = 0; //GamepadStickAngle(_dev, STICK_RIGHT);
    data[7] = 0; //GamepadStickLength(_dev, STICK_RIGHT);
    data[8] = 0; //GamepadTriggerLength(_dev, TRIGGER_LEFT);
    data[9] = 0; //GamepadTriggerLength(_dev, TRIGGER_RIGHT);
    data[10] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_UP);
    data[11] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_DOWN);
    data[12] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_LEFT);
    data[13] = 0; //GamepadButtonDown(_dev, BUTTON_DPAD_RIGHT);
    //data[14] = 0; //GamepadButtonDown(_dev, BUTTON_A); // duct on/off
    data[15] = 0; //GamepadButtonDown(_dev, BUTTON_B);
    data[16] = 0; //GamepadButtonDown(_dev, BUTTON_X);
    data[17] = 0; //GamepadButtonDown(_dev, BUTTON_Y);
    data[18] = 0; //GamepadButtonDown(_dev, BUTTON_BACK);
    data[19] = 0; //GamepadButtonDown(_dev, BUTTON_START);
    data[20] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_SHOULDER);
    data[21] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_SHOULDER);
    data[22] = 0; //GamepadButtonDown(_dev, BUTTON_LEFT_THUMB);
    data[23] = 0; //GamepadButtonDown(_dev, BUTTON_RIGHT_THUMB);
}

void msgCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)                      //Receive topic /cmd_vel and write data[24] to send via TCP/IP
{
    dataInit();
    if (cmd_vel->linear.x > 0){
        data[5] = 1.0;                                                                //Right stick's y-axis move. RU in LabVIEW
    }else if(cmd_vel->linear.x < 0){
        data[5] = -1.0;
    }
    if (std::abs(cmd_vel->linear.x) <= max_angular_vel)
        data[1] = cmd_vel->linear.x/max_linear_vel;                                   //Left sticks's y-axis move. Joy1 Y-axis in LabVIEW;

    if (cmd_vel->angular.z > 0){
        data[8] = 1;                                                                  //LB in LabVIEW
    }else if(cmd_vel->angular.z < 0){
        data[9] = 1;
    }
    if (std::abs(cmd_vel->angular.z) <= max_angular_vel)
        data[0] = cmd_vel->angular.z/max_angular_vel;                                   //Left stick's x-axis move. Joy1 X-axis in LabVIEW

    ROS_INFO("linear_vel.x: %.2f  augular_vel.z: %.2f", cmd_vel->linear.x, cmd_vel->angular.z);
}

void dqnCallback(const std_msgs::Int8::ConstPtr& msg)                                  //Receive topic /action/int8 and write data[24] to send via TCP/IP
{
    dataInit();
    switch (msg->data){
        case 0: // Forward
            data[17] = 1; //Button_Y
            break;
        case 1: // Forward + Right
            data[17] = 1; // Button_Y
            data[15] = 1; // Button_B
            break;
        case 2: // Right
            data[15] = 1; // Button_B
            break;
        case 3: // Backward + Right
            data[15] = 1; // Button_B
            data[14] = 1; // Button_A
            break;
        case 4: // Backward
            data[14] = 1; // Button_A
            break;
        case 5: // Backward + Left
            data[14] = 1; // Button_A
            data[16] = 1; // Button_X
            break;
        case 6: // Left
            data[16] = 1; // Button_X
            break;
        case 7: // Left + Forward
            data[16] = 1; // Button_X
            data[17] = 1; // Button_Y
            break;
        default:
            ROS_INFO("NOT VALID action/int8, out of range (0~7)");
        break;
    }
    ROS_INFO("DQN INPUT ACTION: %d", msg->data);
    Last_dqn = ros::Time::now();                                                            //Save the time
    if (Dqn_start == 0) Dqn_start = 1;                                                      //DQN is running since now
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_sub");
    ros::NodeHandle nh_("~");

    nh_.param("ip_address", ip_address, std::string("143.248.67.21"));                          //TCP/IP target IP address. default : 172.16.0.1
    nh_.param("port", port, 6001);                                                           //TCP/IP target port number. default : 6001
    nh_.param("max_linear_vel", max_linear_vel, 1.0);                                        //Maximum linear, angular velocity of motor
    nh_.param("max_angular_vel", max_angular_vel, 1.0);

    ROS_INFO("ip_address: %s", ip_address.c_str());
    ROS_INFO("port: %d", port);
    ROS_INFO("max_linear_vel: %.2f m/s", max_linear_vel);
    ROS_INFO("max_angular_vel: %.2f rad/s", max_angular_vel);

    ros::Subscriber twist_sub_ = nh_.subscribe("/cmd_vel", 1000, msgCallback);                //Subscriber for the topic "/cmd_vel", "/action/int8" to operate the motor
    ros::Subscriber dqn_sub_ = nh_.subscribe("/action/int8", 1000, dqnCallback);

    fd_set fdset;                                                                             //For time-out of socket
    struct timeval tv;

    c_socket = socket(PF_INET, SOCK_STREAM, 0);                                               //Create the socket for TCP/IP
    fcntl(c_socket, F_SETFL, O_NONBLOCK);
    c_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(port);

    connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr));                             //Try to connect

    FD_ZERO(&fdset);
    FD_SET(c_socket, &fdset);
    tv.tv_sec = 5; tv.tv_usec = 0;                                                              //Set time-out seconds

    if (select(c_socket+1, NULL, &fdset, NULL, &tv) == 1)                                       //True if socket is connected on time
    {                                                                                           //otherwise, terminates the node
        int so_error;
        socklen_t len = sizeof so_error;
        getsockopt(c_socket, SOL_SOCKET, SO_ERROR, &so_error, &len);
        if (so_error == 0){
            printf("Connected. %s:%d is open.\n", ip_address.c_str(), port);
        }
    }
    else
    {
        ROS_INFO("Fail to connect");
        nh_.shutdown();
        return -1;
    }

    while(ros::ok()){
        write(c_socket, data, sizeof(data));                                                      //Send the data via TCP/IP
        ros::Duration(1).sleep();                                                                 //Wait for 1 sec
        ros::spinOnce();

        if (ros::Time::now()-Last_dqn > ros::Duration(2.0) && Dqn_start){
            dataInit();
            write(c_socket, data, sizeof(data));
            ROS_INFO("Time out. It's been two seconds after the last topic came in.");
            ros::Duration(5).sleep();
        }
  }

    close(c_socket);
    return 0;
}
