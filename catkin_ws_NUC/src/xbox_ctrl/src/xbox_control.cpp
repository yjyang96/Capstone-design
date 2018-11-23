#include <stdio.h>
#include <math.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ros/ros.h>
#define _USE_MATH_DEFINES

extern "C" {
	#include "xbox_ctrl/gamepad.h"
}

using namespace std;

#define PORT 4000
#define IPADDR "172.16.0.1"

static const char* button_names[] = {
	"d-pad up",
	"d-pad down",
	"d-pad left",
	"d-pad right",
	"start",
	"back",
	"left thumb",
	"right thumb",
	"left shoulder",
	"right shoulder",
	"???",
	"???",
	"A",
	"B",
	"X",
	"Y"
};

struct tcp_msg{
	float data[4];
	int state;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "XboxCtrl");
	ros::start();
	float vx, vy, wz;
	float motor_max;

	GamepadInit();

	int c_socket;
	struct sockaddr_in c_addr;

	c_socket = socket(PF_INET, SOCK_STREAM, 0);
	printf("socket created\n");
	c_addr.sin_addr.s_addr = inet_addr(IPADDR);
	c_addr.sin_family = AF_INET;
	c_addr.sin_port = htons(PORT);

	if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
		printf("Failed to connect\n");
		close(c_socket);
		return -1;
	}

	tcp_msg tcp_message;
	float data[4];
	bool button[2];
	float motor[4];
	float motor_abs[4];

	while (ros::ok()) {
		bool read_flag;
		read(c_socket, &read_flag, sizeof(bool));
		// cout<<"flag:"<<pickup_flag<<"\n";
		if(!read_flag){
			cout<<"error in ready"<<"\n";
			break;
		}
		GamepadUpdate();

		double dev = GAMEPAD_0;
		GAMEPAD_DEVICE _dev = static_cast<GAMEPAD_DEVICE>(dev);

		data[0] = GamepadStickAngle(_dev, STICK_LEFT); // left stick angle -pi to pi
		data[1] = GamepadStickLength(_dev, STICK_LEFT);  //left stick length 0 to 1
		data[2] = GamepadStickAngle(_dev, STICK_RIGHT);  // right stick angle -pi to pi
		data[3] = GamepadStickLength(_dev, STICK_RIGHT);  //right stick length 0 to 1
		button[0] = GamepadButtonDown(_dev, BUTTON_B);
		button[1] = GamepadButtonDown(_dev, BUTTON_A);
		// cout<<data[0]<<"   "<<data[1]<<"   "<<data[2]<<"   "<<data[3]<<"\n";
		// cout<<button[0]<<"   "<<button[1]<<'\n';

		vx = cos(data[0]) * data[1];
		vy = sin(data[0]) * data[1];
		wz = cos(data[2]) * data[3];

		motor[0] = vx + vy - wz; //right back
		motor[1] = vx - vy - wz; //left back
		motor[2] = vx + vy + wz; //left front
		motor[3] = vx - vy + wz; //right front
		motor[0] *= 60;
		motor[1] *= 60;
		motor[2] *= 60;
		motor[3] *= 60;

		//for motor recalibration
		motor[0] = -motor[0];
		motor[1] = -motor[1];

		for(int i = 0; i < 4 ; i++){
			motor_abs[i] = motor[i] > 0 ? motor[i] : -motor[i];
		}

		motor_max = motor_abs[0];

		for(int i = 1; i<4; i++){
			motor_max = motor_max > motor_abs[i] ? motor_max : motor_abs[i];
		}

		if(motor_max > 60){
			for(int i = 0; i < 4; i++){
				motor[i] /= motor_max;
				motor[i] *= 60;
			}
		}
		
		if(button[0]){
			tcp_message.state=1;
			for(int i = 0; i < 4; i++){
				tcp_message.data[i] = 0;
			}
		}
		else if(button[1]){
			tcp_message.state=2;
			for(int i = 0; i < 4; i++){
				tcp_message.data[i] = 0;
			}
		}
		else{
			tcp_message.state=0;
			for(int i = 0; i < 4; i++){
				tcp_message.data[i] = motor[i];
			}
		}

		write(c_socket, &tcp_message, sizeof(tcp_message));
		if(tcp_message.state==1){
			bool pickup_flag;
			read(c_socket, &pickup_flag, sizeof(bool));
			// cout<<"flag:"<<pickup_flag<<"\n";
			if(!pickup_flag){
				cout<<"error in pickup"<<"\n";
				break;
			}	
		}
		else if(tcp_message.state==2){
			cout<<"string control"<<"\n";
		}
		// cout<<"size:"<<sizeof(tcp_message)<<"\n";
		// cout<<tcp_message.data[0]<<"   "<<tcp_message.data[1]<<"   "<<tcp_message.data[2]<<"   "<<tcp_message.data[3]<<"\n";
		// cout<<"button"<<tcp_message.state<<"\n";
		ros::Duration(0.05).sleep();
	}
	tcp_message.state = 3;
	for(int i = 0; i < 4; i++){
		tcp_message.data[i] = 0;
	}
	write(c_socket, &tcp_message, sizeof(tcp_message));
	close(c_socket);

	return 0;
}
