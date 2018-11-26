#include <ros/ros.h>
#include "core_msgs/ball_position.h"
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <laser_geometry/laser_geometry.h>
#define _USE_MATH_DEFINES
#define UP 0
#define DOWN 1
#define BLUE 0
#define RED 1
#define U_B 0
#define U_R 1
#define D_B 3
#define D_R 4

ros::Publisher tf_pub[6];
ros::Publisher scan_pub;

class msgCallback_balls {
	public:
		int camera_num;// 0 for upper
		int color;// 0 for blue
		tf::TransformListener *listener;
		void transform_ball(const visualization_msgs::Marker& msg); 
		msgCallback_balls(int _camera_num, int _color, tf::TransformListener *_listener){
			camera_num = _camera_num;
			color = _color;
			listener = _listener;
		}
};


void msgCallback_balls::transform_ball(const visualization_msgs::Marker& msg){
	// std::cout<<camera_num<<color<<"what\n";
	tf::StampedTransform transform;
	try{
		if(camera_num== 0){
			listener->lookupTransform("/base_frame", "/camera_link1", ros::Time(0), transform);
		}
		else{
			listener->lookupTransform("/base_frame", "/camera_link2", ros::Time(0), transform);
		}
    }
    catch (tf::TransformException &ex) {
    	std::cout<<"transform error\n";
    	return;
    }


    core_msgs::ball_position new_msg;
	new_msg.size=msg.points.size();

	if(new_msg.size){
        new_msg.img_x.resize(new_msg.size);
        new_msg.img_y.resize(new_msg.size);
        new_msg.img_z.resize(new_msg.size);
        for(int i=0;i<new_msg.size;i++){
			tf::Vector3 output; //declare tf::Vector3. this output will have values of transformed position of ball1
			tf::Vector3 input(msg.points[i].x, msg.points[i].y, msg.points[i].z); //declare tf::Vector3. this input will have values of position of ball1 before trnasformation
			output = transform*input; // apply transformation.
			new_msg.img_x[i] = output.x();
			new_msg.img_y[i] = output.y();
			new_msg.img_z[i] = output.z();
	    }
	}
	// std::cout<<color+camera_num*2<<"      "<<new_msg.size<<"\n";
	tf_pub[color+camera_num*3].publish(new_msg);
}

class msgCallback_lidar {
	public:
		tf::TransformListener *listener;
		laser_geometry::LaserProjection *projector;
		void transform_lidar(const sensor_msgs::LaserScan::ConstPtr& scan_in); 
		msgCallback_lidar(tf::TransformListener *_listener, laser_geometry::LaserProjection *_projector){
			listener = _listener;
			projector = _projector;
		}
};

int circle_num(int num){
	return (num > 359) ? (num - 360) : ((num < 0) ? (num + 360) : num );

}

void msgCallback_lidar::transform_lidar(const sensor_msgs::LaserScan::ConstPtr& scan_in){
	sensor_msgs::PointCloud cloud;
	projector->projectLaser(*scan_in, cloud);	


	tf::StampedTransform transform;
	try{
		listener->lookupTransform("/base_frame", "/laser_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
    	std::cout<<"transform error2\n";
    	return;
    }


    cloud.header.frame_id = "/base_frame";
    for(int i=0;i<cloud.points.size();i++){
		tf::Vector3 output; //declare tf::Vector3. this output will have values of transformed position of ball1
		tf::Vector3 input(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z); //declare tf::Vector3. this input will have values of position of ball1 before trnasformation
		output = transform*input; // apply transformation.
		cloud.points[i].x = output.x();
		cloud.points[i].y = output.y();
		cloud.points[i].z = output.z();
    }


	scan_pub.publish(cloud);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf_ball"); //init ros nodd
	ros::NodeHandle nh; //create node handler
	tf::TransformListener listener;
	laser_geometry::LaserProjection projector;
	
	tf_pub[U_B] = nh.advertise<core_msgs::ball_position>("/blue_tf", 1); //setting publisher
	tf_pub[U_R] = nh.advertise<core_msgs::ball_position>("/red_tf", 1);
	tf_pub[D_B] = nh.advertise<core_msgs::ball_position>("/blue_tf2", 1); //setting publisher
	tf_pub[D_R] = nh.advertise<core_msgs::ball_position>("/red_tf2", 1);
	msgCallback_balls upper_blue(UP, BLUE, &listener);
	msgCallback_balls upper_red(UP,RED ,&listener);
	msgCallback_balls down_blue(DOWN,BLUE, &listener);
	msgCallback_balls down_red(DOWN,RED, &listener);
	ros::Subscriber blue_sub = nh.subscribe("/blue_markers", 1, &msgCallback_balls::transform_ball, &upper_blue);
	ros::Subscriber red_sub = nh.subscribe("/red_markers", 1, &msgCallback_balls::transform_ball, &upper_red);
	ros::Subscriber blue_sub2 = nh.subscribe("/blue_markers2", 1, &msgCallback_balls::transform_ball, &down_blue);
	ros::Subscriber red_sub2 = nh.subscribe("/red_markers2", 1, &msgCallback_balls::transform_ball, &down_red);

	scan_pub = nh.advertise<sensor_msgs::PointCloud>("/scan_points", 1);
	msgCallback_lidar lidar(&listener, &projector);
	ros::Subscriber scan_sub = nh.subscribe("/scan", 1, &msgCallback_lidar::transform_lidar, &lidar);

	ros::spin();
}




