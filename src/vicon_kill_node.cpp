#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include "tf/transform_datatypes.h"
#include <cmath> 

double thresh_x;
double thresh_y;
double thresh_z;
double quad_x;
double quad_y;
double quad_z;

void vicon_callback(const nav_msgs::Odometry::ConstPtr& msg){
	quad_x = msg->pose.pose.position.x;
	quad_y = msg->pose.pose.position.y;
	quad_z = msg->pose.pose.position.z;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "vicon_kill");
	ros::NodeHandle n;
	ros::Rate loop_rate(250);


	n.getParam("/vicon_kill/thresh_x",thresh_x);
	n.getParam("/vicon_kill/thresh_y",thresh_y);
	n.getParam("/vicon_kill/thresh_z",thresh_z);

	std::string quad_name = argv[1];
	std::string quad_topic = "/" + quad_name + "/odom";
	ros::Subscriber vicon_sub = n.subscribe(quad_topic, 1000, vicon_callback);

	ros::Publisher kill_pub = n.advertise<std_msgs::Bool>("kill", 1000);

	std_msgs::Bool to_kill;
	to_kill.data = (bool)false;

	while(ros::ok()){
		if(std::abs(quad_x) > thresh_x && !to_kill.data){
			to_kill.data = true;
			std::cout << "STATE: " << to_kill << std::endl;
		}
		if(std::abs(quad_y) > thresh_y && !to_kill.data){
			to_kill.data = true;
			std::cout << "STATE: " << to_kill << std::endl;
		}
		if(std::abs(quad_z) > thresh_z && !to_kill.data){
			to_kill.data = true;
			std::cout << "STATE: " << to_kill << std::endl;
		}

		//std::cout << "STATE: " << to_kill << std::endl;

		kill_pub.publish(to_kill);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}