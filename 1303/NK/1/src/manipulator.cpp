// This header defines the standard ROS classes
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include <sstream>

int  main (int argc, char **argv) {
	srand (time(NULL));
	
	// Initialize the ROS system.
	ros::init(argc, argv, "manipulator");
	// Establish this program as a ROS node.
	ros::NodeHandle n;
	
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
	ros::Rate loop_rate(1);
	
	 while (ros::ok()) {
		geometry_msgs::Twist vel_msg;
		 
		vel_msg.linear.x = (double)(rand() % 10 + 1)/4.0;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		 
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = (double)(rand() % 10 - 5)/2.0;

		velocity_publisher.publish(vel_msg);
	 }
	
	// Send some output as a log message.
	ROS_INFO_STREAM("Hello, ROS!");
}