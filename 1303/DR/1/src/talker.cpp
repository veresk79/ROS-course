#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace ros;
using geometry_msgs::Twist;

int main(int argc, char **argv)
{
  init(argc, argv, "talker");

  NodeHandle nodeHandler;
  Publisher publisher = nodeHandler.advertise<Twist>("/turtle1/cmd_vel", 1000);
  Rate loop_rate(10);
  float speed = 0;
  float delta = 0.1;

  while (ok())
  {
    speed += delta;

    if (speed > 7 || speed <= 0 )
      delta *= -1;

    Twist msg;
    msg.angular.z = 5;
    msg.linear.x += speed;

    publisher.publish(msg);

    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}