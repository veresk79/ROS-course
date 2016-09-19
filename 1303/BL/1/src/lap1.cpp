#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include <sstream>

int  main (int argc, char **argv) {
	srand (time(NULL));
	
	// Initialize the ROS system.
	ros::init(argc, argv, "manipulator");
	// Establish this program as a ROS node.
	ros::NodeHandle n;

	// Send some output as a log message.
	ROS_INFO_STREAM("Hello! \n ^.^ \n ^.-");

	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
	ros::Rate loop_rate(1);
	
	while (ros::ok()) {
		geometry_msgs::Twist vel_msg; 
		 
		vel_msg.linear.x = 20;
		vel_msg.angular.z = 1;

		velocity_publisher.publish(vel_msg);
		loop_rate.sleep();

		vel_msg.linear.x = 10;		 
		vel_msg.angular.z = 10;

		velocity_publisher.publish(vel_msg);
		loop_rate.sleep();

	 }
}
