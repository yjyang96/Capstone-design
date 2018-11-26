#include <ros/ros.h>
#include <geometry_msgs/Twist.h>                                                 //For msg of the topic "/cmd_vel"
#include <math.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Int8.h>
#define _USE_MATH_DEFINES

#define PORT 4000
#define IPADDR "172.16.0.1"

enum Overall_State {FIND_FRONTIER, PICK_BALL};
enum Rio_State {NOTHING, SORT_LEFT, SORT_RIGHT, DUMP_RIGHT, DUMP_MID, DUMP_LEFT, EXIT};

struct tcp_msg{
	float data[4];
	int rio_state;
};

int c_socket;
struct sockaddr_in c_addr;


struct send_data{
	float vx;
	float vy;
	float wz;
	Rio_State state;
};

struct send_data send_data;
bool ready_flag = false;
Overall_State curr_state;

//subscribing msgs about action, it could be shutdown and re connected.
ros::Subscriber action_sub;

void init_send_data(){
	send_data.vx = 0;
	send_data.vy = 0;
	send_data.wz = 0;
	send_data.state = NOTHING;
}

	//    
    //  ^  +-----------+
    //  |  |           |
    //  |  |           |
    //  vy |    ^-->   |
    //  |  |    |wz|   |
    //  |  |    <--v   |
    //     |           |
    //     +-----------+
	//       -----> vx
void sendRioMessage(float vx, float vy, float wz, Rio_State rio_state){
	// std::cout<<"state:"<<my_state<<"    x:"<<vx<<"  y:"<<vy<<"   z:"<<wz<<std::endl;

	tcp_msg tcp_message;
	float motor[4];
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
	motor[2] = -motor[2];
	motor[3] = -motor[3];

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

	tcp_message.rio_state=rio_state;

	write(c_socket, &tcp_message, sizeof(tcp_message));
	ready_flag = false;

}

void msgCallback_for_mapping(const geometry_msgs::Twist::ConstPtr& cmd_vel)                      //Receive topic /cmd_vel and write data[24] to send via TCP/IP
{
	//  in cmd_vel
    //  ^  +-----------+
    //  |  |           |
    //  |  |           |
    //  vx |    <--^   |
    //  |  |    |wz|   |
    //  |  |    v-->   |
    //     |           |
    //     +-----------+
	//       <----- cy
	init_send_data();
    send_data.vx = -(cmd_vel->linear.y);
    send_data.vy = cmd_vel->linear.x;
    send_data.wz = -(cmd_vel->angular.z);
    // ROS_INFO("motor[0]: %.2f motor[1]: %.2f motor[2]: %.2f motor[3]: %.2f", motor[0], motor[1], motor[2], motor[3]);
    ROS_INFO("linear_vel.x: %.2f linear_vel.y: %.2f augular_vel.z: %.2f", cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->angular.z);
}

void msgCallback_for_picking(const std_msgs::Int8::ConstPtr& action) 
{
	//    
    //  ^  +-----------+
    //  |  |           |
    //  |  |           |
    //  vy |    ^-->   |
    //  |  |    |wz|   |
    //  |  |    <--v   |
    //     |           |
    //     +-----------+
	//       -----> vx
	init_send_data();
	switch(action->data){
		case 0:  // forward
			send_data.vy = 1;
			break;
		case 1: // right
			send_data.vx = 1;
			break;
		case 2: // left
			send_data.vx = -1;
			break;
		case 3: // turn right
			send_data.wz = 1;
			break;
		case 4: // turn left
			send_data.wz = -1;
			break;
		case 5: // forward and sorting to red
			send_data.vy = 1;
			send_data.state = SORT_LEFT;
			break;
		case 6: // forward and sorting to blue
			send_data.vy = 1;
			send_data.state = SORT_RIGHT;
			break;
		default:
			break;
	}
}


int main(int argc, char **argv)
{
	//ros init
	ros::init(argc, argv, "control");
	ros::NodeHandle nh;

    action_sub = nh.subscribe("/cmd_vel", 1, msgCallback_for_mapping);                //Subscriber for the topic "/cmd_vel", "/action/int8" to operate the motor
	action_sub = nh.subscribe("/action/int8", 1, msgCallback_for_picking);
	curr_state = FIND_FRONTIER;

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
	
	init_send_data();

	while(ros::ok()){
		sendRioMessage(send_data.vx, send_data.vy , send_data.wz , send_data.state);
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
	sendRioMessage(0,0,0,EXIT);
	close(c_socket);


	return 0;
}
