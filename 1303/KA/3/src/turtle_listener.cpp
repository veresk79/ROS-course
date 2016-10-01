#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <std_msgs/String.h>

void move(const std::string& trg_frame, const std::string& src_frame, ros::NodeHandle &nh, ros::Publisher &turtle_vel)
{
	tf::TransformListener listener;
	float threshold_x = 0.5f, threshold_y = 0.5f;

  	ros::Rate rate(10.0);
  	while (nh.ok())
  	{
    	tf::StampedTransform transform;
    	try
    	{
      		listener.lookupTransform(trg_frame, src_frame, ros::Time(0), transform);
    	}
    	catch (tf::TransformException &e) 
    	{
      		ROS_ERROR("%s", e.what());
      		ros::Duration(1.0).sleep();
      		continue;
    	}

    	geometry_msgs::Twist vel_msg;
    	vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
    	vel_msg.linear.x = 2.0 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    	turtle_vel.publish(vel_msg);
    
    	if (transform.getOrigin().x() < threshold_x 
    			&& transform.getOrigin().y() < threshold_y)
    	break;

    	rate.sleep();
  	}
}

void cry(ros::NodeHandle &nh, const char* voice)
{
	std_msgs::String s;
	s.data = voice;
  	ros::Publisher pub = nh.advertise<std_msgs::String>("/voice", 100);
 
  	ros::Rate loop_rate(1);
  	for (int i = 0; i < 3; i++)
    {
        pub.publish(s);
        loop_rate.sleep();
    }
}

void search(ros::NodeHandle &nh, ros::Publisher &turtle_vel)
{
	move("/turtle2", "/turtle1", nh, turtle_vel);
	cry(nh, "/turtle2");	
}

void back_home(ros::NodeHandle &nh, ros::Publisher &turtle_vel)
{
	ros::NodeHandle nh2;
	move("/turtle2", "world", nh2, turtle_vel);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_turtle_listener");

  ros::NodeHandle nh;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = nh.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);
  search(nh, turtle_vel);
  back_home(nh, turtle_vel);
  
  return 0;
};
