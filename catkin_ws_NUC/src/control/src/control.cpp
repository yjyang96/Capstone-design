#include <ros/ros.h>
#include <geometry_msgs/Twist.h>                                                 //For msg of the topic "/cmd_vel"
#include <math.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#define _USE_MATH_DEFINES

#define PORT 4000
#define IPADDR "172.16.0.1"

struct tcp_msg{
	float data[4];
	int state;
};

int c_socket;
struct sockaddr_in c_addr;

float send_data[4] = {0,0,0,0};
float motor[4] = {0,0,0,0};
bool ready_flag = false;

void sendRioMessage(float vx, float vy, float wz, int rio_state){
	// std::cout<<"state:"<<my_state<<"    x:"<<vx<<"  y:"<<vy<<"   z:"<<wz<<std::endl;

	tcp_msg tcp_message;
	// float motor[4];
	float motor_abs[4];

	motor[0] = vx + vy - wz * 0.64; //right back
	motor[1] = vx - vy - wz * 0.64; //left back
	motor[2] = vx + vy + wz * 0.64; //left front
	motor[3] = vx - vy + wz * 0.64; //right front
	motor[0] /= 0.075 * 2 * M_PI;
	motor[1] /= 0.075 * 2 * M_PI;
	motor[2] /= 0.075 * 2 * M_PI;
	motor[3] /= 0.075 * 2 * M_PI;
	motor[0] *= 30;
	motor[1] *= 30;
	motor[2] *= 30;
	motor[3] *= 30;

	//for motor recalibration
	motor[0] = -motor[0];
	motor[1] = -motor[1];


	for(int i = 0; i < 4; i++){
		motor_abs[i] = motor[i] > 0 ? motor[i] : -motor[i];
	}

	float motor_max = motor_abs[0];

	for(int i = 1; i < 4; i++){
		motor_max = motor_max > motor_abs[i] ? motor_max : motor_abs[i];
	}

	//normalize
	if(motor_max > 60){
		for(int i = 0; i < 4; i++){
			motor[i] /= motor_max;
			motor[i] *= 60;
		}
	}
	
	// update data
	for(int i = 0; i<4; i++){
		tcp_message.data[i] = motor[i];
	}
	

	// stop
	// for(int i = 0; i < 4; i++){
	// 	tcp_message.data[i] = 0;
	// }


	tcp_message.state=rio_state;

	

	write(c_socket, &tcp_message, sizeof(tcp_message));
	ready_flag = false;

}

void msgCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)                      //Receive topic /cmd_vel and write data[24] to send via TCP/IP
{
    send_data[0] = -(cmd_vel->linear.y);
    send_data[1] = cmd_vel->linear.x;
    send_data[2] = -(cmd_vel->angular.z);
    // ROS_INFO("motor[0]: %.2f motor[1]: %.2f motor[2]: %.2f motor[3]: %.2f", motor[0], motor[1], motor[2], motor[3]);
    ROS_INFO("linear_vel.x: %.2f linear_vel.y: %.2f augular_vel.z: %.2f", cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->angular.z);
}

int main(int argc, char **argv)
{
	//ros init
	ros::init(argc, argv, "control");
	ros::NodeHandle nh;

    ros::Subscriber twist_sub = nh.subscribe("/cmd_vel", 2, msgCallback);                //Subscriber for the topic "/cmd_vel", "/action/int8" to operate the motor

	// socket open start
	c_socket = socket(PF_INET, SOCK_STREAM, 0);
	std::cout<<"socket created\n";
	c_addr.sin_addr.s_addr = inet_addr(IPADDR);
	c_addr.sin_family = AF_INET;
	c_addr.sin_port = htons(PORT);

	// ros::spin();

	if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
		std::cout<<"Failed to connect\n";
		close(c_socket);
		return -1;
	}
	// socket connect end
	send_data[0]= 0;
	send_data[1]=0;
	send_data[2]=0;
	send_data[3]=0;

	while(ros::ok()){
		sendRioMessage(send_data[0], send_data[1] , send_data[2] , 0);
		if(!ready_flag){
			read(c_socket, &ready_flag, sizeof(bool));
			if(!ready_flag){
				std::cout<<"msg not ready\n";
				ros::shutdown();
			}
		}


		ros::Duration(0.07).sleep();
		ros::spinOnce();
	}
	sendRioMessage(0,0,0,3);
	close(c_socket);


	return 0;
}
