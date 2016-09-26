#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv) {

	ros::init(argc, argv, "lab1");
	ros::NodeHandle node;

	ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(1);

	srand(time(NULL));
	while(ros::ok()) {
		geometry_msgs::Twist msg;
		msg.linear.x = (double) (rand()%10+1)/4.0;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = (double) (rand()%10-5)/2.0;
		
		publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
