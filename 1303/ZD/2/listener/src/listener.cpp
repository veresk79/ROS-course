#include <ros/ros.h>
#include <message/message.h>
#include <math.h>

message::message::_salary_type maxKnownSalary = -1;
message::message::_salary_type mySalary = -1;
std::string myName;

void listenerCallback(const message::message & msg)
{
  message::message::_salary_type salary = msg.salary;
  if(salary > maxKnownSalary) {
    maxKnownSalary = salary;
  }

  if(myName.compare(msg.name)==0) {
    mySalary = salary;
  }

  if(msg.isLast) {
    if(mySalary==maxKnownSalary){
      ROS_INFO("I am %s. My salary is %ld !", myName.c_str(), mySalary);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  std::string fullNodeName = ros::this_node::getName();
  myName = fullNodeName.substr(1, fullNodeName.length() - 1);
  ROS_INFO("%s is listening...", myName.c_str());
  ros::NodeHandle nh;
  ros::Subscriber announcementsSubscriber = nh.subscribe("announcements", 100, listenerCallback);
  ros::spin();
  return 0;
}
