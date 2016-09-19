#include <ros/ros.h>
#include <message/message.h>
#include <math.h>

typedef std::pair< std::string,long int > workerAndSalary;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "announcer");
  ROS_INFO("Announcer started...");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<message::message>("announcements", 100);

  ros::Rate loop_rate(1);

  std::vector< workerAndSalary > workers;

  workers.push_back(workerAndSalary("Ivanov", 2000));
  workers.push_back(workerAndSalary("Petrov", 5000));
  workers.push_back(workerAndSalary("Sidorov", 8000));
  sleep(1);

  for (unsigned int i = 0; i < workers.size(); i++) {
    message::message msg;
    msg.name = workers[i].first;
    msg.salary = workers[i].second;
    msg.isLast = (i == (workers.size() - 1));
    pub.publish(msg);
    ROS_INFO("Sent info about %s",workers[i].first.c_str());
    ros::spinOnce();
    loop_rate.sleep();
  }
}
