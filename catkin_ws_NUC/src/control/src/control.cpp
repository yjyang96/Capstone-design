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


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include "core_msgs/ball_position.h"


#include "sensor_msgs/PointCloud.h"


#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>


#include "core_msgs/markerXY.h"

#define _USE_MATH_DEFINES

#define PORT 4000
#define IPADDR "172.16.0.1"

enum Overall_State {FIND_FRONTIER, PICK_BALL, GO_STARTPOS,										\
					GO_RIGHT_GOAL_MARKER, GO_RIGHT_GOAL_LIDAR, DROP_RIGHT, GO_STARTPOS_AGAIN,	\
					GO_LEFT_GOAL_MARKER, GO_LEFT_GOAL_LIDAR, DROP_LEFT};
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
	// check myrio is ready
	if(!ready_flag){
		read(c_socket, &ready_flag, sizeof(bool));
		if(!ready_flag){
			ROS_INFO("msg not ready");
			ros::shutdown();
		}
	}

	// std::cout<<"state:"<<my_state<<"    x:"<<vx<<"  y:"<<vy<<"   z:"<<wz<<std::endl;

	tcp_msg tcp_message;
	float motor[4];
	float motor_abs[4];

	motor[0] = vx + vy - wz * 0.64; //right back
	motor[1] = vx - vy - wz * 0.64; //left back
	motor[2] = vx + vy + wz * 0.64; //left front
	motor[3] = vx - vy + wz * 0.64; //right front

	for(int i = 0; i < 4; i++){
		motor[i] *= 60 * (40.0/15.0); //  60 for round per min 40/15 for gear ratio
		motor[i] /= 0.075 * 2 * M_PI;  //0.075 for  radius, 2pi for radian to round
		motor[i] /= 6; // 6 for my mind, just constant we need to modify
	}

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
	ROS_INFO("motor[0]: %.2f motor[1]: %.2f motor[2]: %.2f motor[3]: %.2f", motor[0], motor[1], motor[2], motor[3]);

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
	//       <----- vy
	init_send_data();
    send_data.vx = -(cmd_vel->linear.y);
    send_data.vy = cmd_vel->linear.x;
    send_data.wz = -(cmd_vel->angular.z);
    // ROS_INFO("motor[0]: %.2f motor[1]: %.2f motor[2]: %.2f motor[3]: %.2f", motor[0], motor[1], motor[2], motor[3]);
    ROS_INFO("linear_vel.x: %.2f linear_vel.y: %.2f augular_vel.z: %.2f", cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->angular.z);
}

int blue_ball_remain = 3;
int red_ball_remain = 3;
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
	ROS_INFO("signal %d", action->data);
	switch(action->data){
		case 0:  // forward
			send_data.vy = 0.10;
			break;
		case 1: // right
			send_data.vx = 0.2;
			break;
		case 2: // left
			send_data.vx = -0.2;
			break;
		case 3: // turn right
			send_data.wz = 0.3;
			break;
		case 4: // turn left
			send_data.wz = -0.3;
			break;
		case 5: // forward and sorting to 
		case 6: // forward and sorting to blue
			send_data.vy = 0.10;
			red_ball_remain--;
			if(action->data == 5){
				ROS_INFO("pick red");
				send_data.state = SORT_RIGHT;
			}
			else{
				ROS_INFO("pick blue");
				send_data.state = SORT_LEFT;
			}
			break;
		default:
			ROS_INFO("wrong action");
			break;
	}
	// ROS_INFO("action num : %d", action->data);
}


//------------------------------------------------------------------------
//for detect ball check


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

//------------------------------------------------------------------------
// for get scan data

float coeffi[2] = {0,0}; //a, b y=ax+b for wall
bool is_new_coeffi = false;

void get_scan(const sensor_msgs::PointCloud& points){
	std::vector<geometry_msgs::Point32> positions;
	int n = 20;
	for(int i =0; i < points.points.size(); i++){
		if(points.channels[1].values[i] <= n || points.channels[1].values[i] >= 360 - n){ //back is 0 degree
			positions.push_back(points.points[i]);
		}
	}
	
	// float a, b;
	float x = 0;
	float y = 0;
	float xy = 0;
	float xx = 0;
	for(int i =0; i<positions.size();i++){
		// -------------------------------------------------------------------------
		// i change x,y because cord of xy is change between capstone 1 and 2
		x += positions[i].y;
		y += positions[i].x;
		xy += positions[i].y * positions[i].x;
		xx += positions[i].y * positions[i].y;
		// -------------------------------------------------------------------------
	}

	coeffi[0] = (float(positions.size()) * xy - x * y)/(float(positions.size())*xx - x*x);
	coeffi[1] = (xx*y-x*xy)/(float(positions.size())*xx-x*x);

	is_new_coeffi = true;
	// std::cout<<coeffi[0]<<"       "<<coeffi[1]<<"\n";

	// sensor_msgs::PointCloud test;
	// test.header = points.header;
	// test.points.resize(positions.size());
	// for(int i= 0; i< positions.size();i++){
	// 	test.points[i] = positions[i];
	// }
	// pub_test.publish(test);

}

//--------------------------------------------------------------------------------

geometry_msgs::Pose robot_pos;

//  in cmd_vel
    //  ^  +-----------+
    //  |  |           |
    //  |  |           |
    //  x  |     O     |
    //  |  |    OzO    |
    //  |  |     O     |
    //     |           |
    //     +-----------+
	//       <----- y

void get_robot_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)                      //Receive topic /cmd_vel and write data[24] to send via TCP/IP
{
	robot_pos = msg->pose;
}

//-----------------------------------------------------------------------------------
//get goal_status
int goal_status = 0;
void get_goal_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)                      //Receive topic /cmd_vel and write data[24] to send via TCP/IP
{
	goal_status = msg->status_list.front().status;

    // Check if any status is 3. If so, write it to goal_status ?
    // for (auto const& i : msg->status_list) {
    //     if (i.status == SUCCEEDED)
    //         goal_status = SUCCEEDED;
    // }
}

//-----------------------------------------------------------------------------------
//for CNN
Rio_State is_catdog;
void msgCallback_iscatdog(const std_msgs::Int8::ConstPtr& action) 
{
	switch(action->data){
		case 0:
			is_catdog = DUMP_LEFT;
			break;
		case 1:
			is_catdog = DUMP_RIGHT;
			break;
		default:
			ROS_INFO("no case for catdog");
			break;
	}
}

float markerX_mid = NAN;
float markerX_diff = NAN;  // 22.5 at start point, 110 at close goal
void msgCallback_markerXY(const core_msgs::markerXY::ConstPtr& msg){

	//only NAN == NAN is false
	if(msg->marker1X[0] == msg->marker1X[0]){
		markerX_mid = 0;
		markerX_diff = 0;
		for(int i = 0 ; i<4; i++){
			markerX_mid += msg->marker1X[i];
			markerX_diff += (i == 1 || i == 2) ? msg->marker1X[i] : -(msg->marker1X[i]);
		}
		markerX_mid /= 4;
		markerX_diff /= 4;
	}
	else if(msg->marker2X[0] == msg->marker2X[0]){
		markerX_mid = 0;
		markerX_diff = 0;
		for(int i = 0 ; i<4; i++){
			markerX_mid += msg->marker2X[i];
			markerX_diff += (i == 1 || i == 2) ? msg->marker2X[i] : -(msg->marker2X[i]);
		}
		markerX_mid /= 4;
		markerX_diff /= 4;
	}
	else{
		markerX_mid = NAN;
		markerX_diff = NAN;
	}

}
//--------------------------------------------------------------------------------------



int main(int argc, char **argv)
{
	//ros init
	ros::init(argc, argv, "control");
	ros::NodeHandle nh;



	ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
	ros::Publisher goal_cancle = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);
	ros::Subscriber goal_status_sub = nh.subscribe("/move_base/goal_status",1, get_goal_status);
    
	ros::Subscriber action_sub = nh.subscribe("/cmd_vel", 1, msgCallback_for_mapping);                //Subscriber for the topic "/cmd_vel", "/action/int8" to operate the motor
	// ros::Subscriber action_sub = nh.subscribe("/action/int8", 1, msgCallback_for_picking);
	ros::Subscriber dump_catdog;
	// ros::Subscriber dump_catdog = nh.subscribe("/iscatdog/int8", 1, msgCallback_iscatdog);
	
	ros::Subscriber markerXY_sub;
	// ros::Subscriber markerXY_sub = nh.subscribe("/markerXY", 1, msgCallback_markerXY);
	curr_state = FIND_FRONTIER;
	// curr_state = PICK_BALL;

	msgCallback_balls upper_blue(UP, BLUE);
	msgCallback_balls upper_red(UP, RED);

    ros::Subscriber blue_sub = nh.subscribe("/blue_tf", 1, &msgCallback_balls::get_pos, &upper_blue);
	ros::Subscriber red_sub = nh.subscribe("/red_tf", 1, &msgCallback_balls::get_pos, &upper_red);

	ros::Subscriber slam_out_pose = nh.subscribe("/slam_out_pose", 1, get_robot_pos);

	ros::Subscriber scan_sub;
	// ros::Subscriber scan_sub = nh.subscribe("scan_points", 1, &get_scan);


	// socket open start
	c_socket = socket(PF_INET, SOCK_STREAM, 0);
	ROS_INFO("socket created");
	c_addr.sin_addr.s_addr = inet_addr(IPADDR);
	c_addr.sin_family = AF_INET;
	c_addr.sin_port = htons(PORT);

	// ros::spin();

	if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
		ROS_INFO("Failed to connect");
		close(c_socket);
		return -1;
	}
	// socket connect end
	
	init_send_data();

	
	int state_debug = 0;
	while(ros::ok()){
		state_debug ++;
		if(state_debug > 9){
			ROS_INFO("cur state %d", curr_state);
			state_debug = 0;
		}
		switch(curr_state){
			case FIND_FRONTIER:{
				if((balls_pos[UP_BLUE].size() + balls_pos[UP_RED].size() > 0  && robot_pos.position.x > 1.5) || robot_pos.position.x > 2.5){
					//shutdown find_frontier_node
					system("rosnode kill find_frontier_node");

					actionlib_msgs::GoalID cancle_msg;
					goal_cancle.publish(cancle_msg);
					curr_state = PICK_BALL;
					ROS_INFO("change state to PICK BALL");
					action_sub.shutdown();
					action_sub = nh.subscribe("/action/int8", 1, msgCallback_for_picking);
					init_send_data();
				}
				sendRioMessage(send_data.vx, send_data.vy, send_data.wz, send_data.state);
				break;
			}
			case PICK_BALL:{
				// if (red_ball_remain == 0 && blue_ball_remain == 0){
					// sendRioMessage(send_data.vx, send_data.vy, send_data.wz, send_data.state);
					
					// ROS_INFO("start final picking");
					// //wait for final picking
					// ros::Duration(5).sleep();

					action_sub.shutdown();
					action_sub = nh.subscribe("/cmd_vel", 1, msgCallback_for_mapping);
					ROS_INFO("change state to GO_STARTPOS");
					curr_state = GO_STARTPOS;
				// }
				// sendRioMessage(send_data.vx, send_data.vy, send_data.wz, send_data.state);
				// ROS_INFO("remain ball %d", blue_ball_remain + red_ball_remain);
				// if(send_data.state == SORT_LEFT || send_data.state == SORT_RIGHT){
				// 	ROS_INFO("pick something");
				// 	ROS_INFO("pick something");
				// 	ROS_INFO("pick something");
				// 	ROS_INFO("pick something");
				// 	ros::Duration(3).sleep();
				// }
				// curr_state = GO_STARTPOS;
				break;
			}
			case GO_STARTPOS:{
				// ROS_INFO("goal_status %d", goal_status);
				ROS_INFO("orientation %.2f", robot_pos.orientation.z);
				if (-0.07 < robot_pos.position.x && robot_pos.position.x < 0.07 &&
					-0.07 < robot_pos.position.y && robot_pos.position.y < 0.07 &&
					sin(M_PI_4) - 0.05 < robot_pos.orientation.z && robot_pos.orientation.z < sin(M_PI_4) + 0.05 ){
				// if (goal_status == 3){
					actionlib_msgs::GoalID cancle_msg;
					goal_cancle.publish(cancle_msg);
					curr_state = GO_RIGHT_GOAL_MARKER;
					ROS_INFO("change state to GO RIGHT GOAL MARKER");
					action_sub.shutdown();
					markerXY_sub = nh.subscribe("/markerXY", 1, msgCallback_markerXY);
					dump_catdog = nh.subscribe("/iscatdog/int8", 1, msgCallback_iscatdog);
					scan_sub = nh.subscribe("scan_points", 1, &get_scan);
				}
				else{
					geometry_msgs::PoseStamped goal_position;
					goal_position.header.frame_id = "map";
					goal_position.header.stamp = ros::Time::now();
					goal_position.pose.position.x = 0;
					goal_position.pose.position.y = 0;
					tf::Quaternion goal_orientation = tf::createQuaternionFromYaw(M_PI_2);
					tf::quaternionTFToMsg(goal_orientation, goal_position.pose.orientation);
					goal_pub.publish(goal_position);
				}
				sendRioMessage(send_data.vx, send_data.vy, send_data.wz, send_data.state);
				break;
			}
			case GO_RIGHT_GOAL_MARKER:
			case GO_LEFT_GOAL_MARKER:{
				//escape didn't get message
				//only NAN != NAN is ture
				if(markerX_diff != markerX_diff){
					break;
				}
				init_send_data();
				ROS_INFO("2   mid %.2f, diff %.2f", markerX_mid, markerX_diff);
				if(markerX_mid < 340 && markerX_mid > 300){
					if(markerX_diff > 80){
						curr_state = (curr_state == GO_RIGHT_GOAL_MARKER) ? GO_RIGHT_GOAL_LIDAR : GO_LEFT_GOAL_LIDAR;
					}
					else{
						send_data.vy = -0.1;
					}
				}
				else{
					if(markerX_diff > 80){
						send_data.vx = markerX_mid - 320 > 0 ? -0.05 : 0.05;
					}
					else{
						send_data.vy = -0.1;
						send_data.vx = markerX_mid - 320 > 0 ? -0.05 : 0.05;
					}
				}
				markerX_diff = NAN;
				markerX_mid = NAN;
				sendRioMessage(send_data.vx, send_data.vy, send_data.wz, send_data.state);
				break;
			}
			case GO_RIGHT_GOAL_LIDAR:
			case GO_LEFT_GOAL_LIDAR:{
				if(!is_new_coeffi){
					break;
				}

				ROS_INFO("slop %.2f, dist %.2f", coeffi[0], coeffi[1]);
				init_send_data();
				if(-0.1 < coeffi[0]  && coeffi[0] < 0.1){
					if(-0.47 < coeffi[1]  && coeffi[1] < -0.44){
						ROS_INFO("DROP");
						curr_state = (curr_state == GO_RIGHT_GOAL_LIDAR) ? DROP_RIGHT : DROP_LEFT;
					}
					else{
						send_data.vy = coeffi[1] + 0.455 > 0 ? 0.05 : -0.05;
					}
				}
				else{
					if(-0.47 < coeffi[1]  && coeffi[1] < -0.44){
						send_data.wz = coeffi[0] > 0 ? 0.05 : -0.05;
					}
					else{
						send_data.vy = coeffi[1] + 0.455 > 0 ? 0.05 : -0.05;
						send_data.wz = coeffi[0] > 0 ? 0.05 : -0.05;
					}
				}
				is_new_coeffi = false;
				sendRioMessage(send_data.vx, send_data.vy, send_data.wz, send_data.state);
				break;
			}
			case DROP_RIGHT:
			case DROP_LEFT:{
				init_send_data();
				send_data.state = is_catdog;
				sendRioMessage(send_data.vx, send_data.vy, send_data.wz, send_data.state);
				ROS_INFO("dump start");
				ros::Duration(5).sleep();
				ROS_INFO("dump end");
				if (curr_state == DROP_RIGHT){
					init_send_data();
					sendRioMessage(0,0,0,DUMP_MID);
					curr_state = GO_STARTPOS_AGAIN;
					ROS_INFO("go startpos again");
				}
				else{
					//end
					init_send_data();
					sendRioMessage(0,0,0,DUMP_MID);
					ros::shutdown();
				}
				break;
			}
			case GO_STARTPOS_AGAIN:{
				ROS_INFO("orientation %.2f", robot_pos.orientation.z);
				// if (-0.07 < robot_pos.position.x && robot_pos.position.x < 0.07 &&
				// 	-0.07 < robot_pos.position.y && robot_pos.position.y < 0.07 &&
				// 	sin(-M_PI_4) - 0.05 < robot_pos.orientation.z && robot_pos.orientation.z < sin(-M_PI_4) + 0.05 ){
				// // if (goal_status == 3){
				// 	actionlib_msgs::GoalID cancle_msg;
				// 	goal_cancle.publish(cancle_msg);
				// 	curr_state = GO_RIGHT_GOAL_MARKER;
				// 	ROS_INFO("change state to GO RIGHT GOAL MARKER");
				// 	action_sub.shutdown();
				// 	markerXY_sub = nh.subscribe("/markerXY", 1, msgCallback_markerXY);
				// 	dump_catdog = nh.subscribe("/iscatdog/int8", 1, msgCallback_iscatdog);
				// 	scan_sub = nh.subscribe("scan_points", 1, &get_scan);
				// }
				// else{
				// 	geometry_msgs::PoseStamped goal_position;
				// 	goal_position.header.frame_id = "map";
				// 	goal_position.header.stamp = ros::Time::now();
				// 	goal_position.pose.position.x = 0;
				// 	goal_position.pose.position.y = 0;
				// 	tf::Quaternion goal_orientation = tf::createQuaternionFromYaw(M_PI_2);
				// 	tf::quaternionTFToMsg(goal_orientation, goal_position.pose.orientation);
				// 	goal_pub.publish(goal_position);
				// }
				// sendRioMessage(send_data.vx, send_data.vy, send_data.wz, send_data.state);
				break;
			}
			default:{
				ROS_INFO("error overall state, non defined");
			}
		}
		// ROS_INFO("vx: %.2f vy: %.2f wz: %.2f state: %d", send_data.vx, send_data.vy, send_data.wz, send_data.state);
		


		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}
	sendRioMessage(0,0,0, EXIT);
	close(c_socket);


	return 0;
}
