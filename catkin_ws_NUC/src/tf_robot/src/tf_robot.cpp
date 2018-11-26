#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#define _USE_MATH_DEFINES



void msgCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)                      //Receive topic /cmd_vel and write data[24] to send via TCP/IP
{
	static tf::TransformBroadcaster br;
    tf::Quaternion q;
	tf::Transform transform;  //define transform

	transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );  //set the translation related variables
	// q = tf::Quaternion::Quaternion(pose->pose.orientation.x,pose->pose.orientation.y,pose->pose.orientation.z,pose->pose.orientation.w);
	tf::quaternionMsgToTF(msg->pose.orientation, q);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_frame"));  //publish tf. parent link is "world" and child link is "camera_link"
}

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf");  //init ROS node
	ros::NodeHandle nh;
	ros::Rate r(10);  //10 hz
	static tf::TransformBroadcaster br;  //set transformbroadcaster (similar to publisher)
	tf::Quaternion q;
	tf::Transform transform;  //define transform
	ros::Subscriber slam_out_pose = nh.subscribe("/slam_out_pose", 2, msgCallback);

	transform.setOrigin( tf::Vector3(0, 0, 0) );  //set the translation related variables
	q.setRPY(0, 0, 0);  //set the rotation related variables here
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_frame"));  //publish tf. parent link is "world" and child link is "camera_link"
	

	while(ros::ok()){
		
		// for camera axis
		// ---------x------>
		// |
		// |       X X
		// z        y
		// |       X X
		// |
		// |
		// v

		// camera1
		transform.setOrigin( tf::Vector3(0.140, 0, 0.270) );  //set the translation related variables 20
		q.setRPY(-M_PI_2 -0.30, 0, -M_PI_2 );  //set the rotation related variables here
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_frame", "/camera_link1"));  //publish tf. parent link is "world" and child link is "camera_link"

		//camera2
		transform.setOrigin( tf::Vector3(-0.150, 0, 0.200) );  //set the translation related variables
		q.setRPY(-M_PI_2, 0, M_PI_2);  //set the rotation related variables here 5
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_frame", "/camera_link2"));  //publish tf. parent link is "world" and child link is "camera_link"

		//laser
		transform.setOrigin( tf::Vector3(-0.040, -0.0475, 0.335) );  //set the translation related variables
		q.setRPY(0, 0, 0);  //set the rotation related variables here 5
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_frame", "/laser_frame"));  //publish tf. parent link is "world" and child link is "camera_link"

		r.sleep();
		ros::spinOnce();
	}

	return 0;
}

