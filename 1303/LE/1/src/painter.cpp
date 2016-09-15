#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "painter");
  ros::NodeHandle nodeHandle;
  const int queueSize = 1000;
  ros::Publisher publisher = nodeHandle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", queueSize);

  ros::Rate rate(1);
  while(ros::ok()) {
    geometry_msgs::Twist msg;
    msg.linear.x = 2.04;
    msg.angular.z = 1.8;

    publisher.publish(msg);

    rate.sleep();
    msg.linear.x = 12.0;
    msg.angular.z = 5.7;
    publisher.publish(msg);
    rate.sleep();
  }
  return 0;
}