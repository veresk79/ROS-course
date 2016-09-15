#include "ros/ros.h" 
#include "turtlesim/Pose.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h> 
#include <time.h>
#include <math.h>

ros::Publisher send_point;
ros::Subscriber get_point;

turtlesim::Pose next_pos;
turtlesim::Pose current_pos;

void getCurrentPoint(const turtlesim::Pose::ConstPtr& pose_message);
void generateNextPoint();
void rotateTurtle();
void moveTurtle();

int main(int argc, char **argv)
{
  srand (time(NULL));
  
  ros::init(argc, argv, "turtle_command");
  ros::NodeHandle n;
  
  send_point = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  get_point = n.subscribe("/turtle1/pose", 1, getCurrentPoint);
  
  ros::Rate loop_rate(1);
  while(ros::ok()) {
		geometry_msgs::Twist message;
		message.linear.x = 5;
		message.linear.y = 0;
		message.linear.z = 0;
		message.angular.x = 0;
		message.angular.y = 0;
		message.angular.z = (double)(rand())/RAND_MAX*(3 - (-3)) + (-3);
		send_point.publish(message);	
		ros::spinOnce();
		loop_rate.sleep();
  }
  ros::spin();
  return 0;
}

void getCurrentPoint(const turtlesim::Pose::ConstPtr & pose_message) {
	current_pos.x=pose_message->x;
	current_pos.y=pose_message->y;
	current_pos.theta=pose_message->theta;
	std::cout << "turtle " << current_pos.x << " " << current_pos.y << " " << current_pos.theta << std::endl;
	ros::spinOnce();
}

void generateNextPoint() {
	next_pos.x = rand() % 9;
	next_pos.y = rand() % 9;
}

void rotateTurtle() {
	double angle = 0;
	if (current_pos.y == next_pos.y) {
		angle = 0;
	} else if (current_pos.x == next_pos.x) {
		angle = 1.5708;
	} else {
		angle = atan(abs(current_pos.y - next_pos.y)/abs(current_pos.x - next_pos.x));
	}
	
	geometry_msgs::Twist message;
	message.linear.x = 0;
	message.linear.y = 0;
	message.linear.z = 0;
	message.angular.x = 0;
	message.angular.y = 0;
	
	if (current_pos.theta < angle) {
		message.angular.z = -0.02;
	} else {
		message.angular.z = 0.02;
	}
	
	
	std::cout << "angle=" << angle << " " << current_pos.theta << std::endl;
	while (abs(current_pos.theta - angle) < 0.09) {
		send_point.publish(message);
		ros::spinOnce();
		std::cout << angle << " " << current_pos.theta << std::endl;
	}
	message.angular.z = 0;
	send_point.publish(message);
}

double getDistance(){
	return sqrt(pow((abs(next_pos.x-current_pos.x)),2)+pow((abs(next_pos.y-current_pos.y)),2));
}

void moveTurtle() {
	/*if (getDistance() < 1.5) {
		geometry_msgs::Twist message;
		message.linear.x = 0;
		message.linear.y = 0;
		message.linear.z = 0;
		message.angular.x = 0;
		message.angular.y = 0;
		message.angular.z = 0;
		send_point.publish(message);
		ros::spinOnce();
		move_status = false;
		std::cout << "1" << std::endl;
	}*/
	
	//if (!move_status) {
	//	move_status = true;
		//generateNextPoint();
		//rotateTurtle();	
			
		/*geometry_msgs::Twist message;
		message.linear.x = 3;
		message.linear.y = 0;
		message.linear.z = 0;
		message.angular.x = 0;
		message.angular.y = 0;
		message.angular.z = 0;
		send_point.publish(message);
		ros::spinOnce();
	//	std::cout << "2" << std::endl;
	//}*/
	//std::cout << "turtle " << current_pos.x << " " << current_pos.y << std::endl;
	//std::cout << "goal " << next_pos.x << " " << next_pos.y << std::endl;
}
