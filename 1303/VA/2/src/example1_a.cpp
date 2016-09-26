#include <ros/ros.h>
#include "turtlesim/Color.h"
#include "turtlesim/Pose.h"

void color_callback(const turtlesim::Color::ConstPtr& col){
ROS_INFO("red: %d, green: %d, blue: %d", col->r, col->g, col->b);
}

void pase_callback(const turtlesim::Pose& msgIn){
ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "position=(" << msgIn.x << "," << msgIn.y << ")" << "direction=" << msgIn.theta);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example1_a");
  ros::NodeHandle node;
  ros::Subscriber subscribe_color = node.subscribe("/tuetle1/color_sensor", 1000, color_callback);
  ros::Subscriber subscribe_pose = node.subscribe("/tuetle1/pose", 1000, color_callback);
  ros::spin();
  return 0;
}
