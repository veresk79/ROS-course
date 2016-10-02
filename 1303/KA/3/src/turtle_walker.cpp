#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <std_msgs/String.h>
#include <ros/console.h>

std::string turtle_name;
std::string rescuer_turtle;
bool isFollowing = false;

void walk(std::string frame_name)
{
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(turtle_name + "/cmd_vel", 1);
  
  geometry_msgs::Twist command;
  
  ros::Rate loop_rate(1);
  while(ros::ok() && !isFollowing) 
  {
  	command.linear.x = (double) (rand() % 10 + 1) / 4.0;
	command.angular.z = (double) (rand() % 10 - 5) / 2.0;
  
	pub.publish(command);
	ros::spinOnce();
	loop_rate.sleep();
  }
}

void move(std::string source_frame, std::string target_frame, ros::Publisher &turtle_vel)
{
	tf::TransformListener listener;
	float threshold_x = 0.01f, threshold_y = 0.01f;

	ros::NodeHandle nh;
  	ros::Rate rate(10.0);
  	while (nh.ok())
  	{
    	tf::StampedTransform transform;
    	try
    	{
      		listener.lookupTransform(source_frame, target_frame, ros::Time(0), transform);
    	}
    	catch (tf::TransformException &e) 
    	{
      		ROS_ERROR("%s", e.what());
      		ros::Duration(1.0).sleep();
      		continue;
    	}

    	geometry_msgs::Twist vel_msg;
    	vel_msg.angular.z = 3.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
    	vel_msg.linear.x = 1.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    	turtle_vel.publish(vel_msg);
    
    	if (transform.getOrigin().x() < threshold_x 
    			&& transform.getOrigin().y() < threshold_y)
			break;
		
    	rate.sleep();
  	}
}

void follow(std::string source_frame, std::string target_frame)
{
  ros::NodeHandle nh;
  ros::Publisher turtle_vel = nh.advertise<geometry_msgs::Twist>(source_frame + "/cmd_vel", 1);
  move(source_frame, target_frame, turtle_vel);
}


void handlerVoice(const std_msgs::String& str)
{
  	rescuer_turtle = str.data;
  	isFollowing = true;
}

ros::Subscriber listen()
{
  ros::NodeHandle nh;
  return nh.subscribe("/voice", 100, &handlerVoice);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_turtle_walker");
  
  if (argc != 2) 
  {
  	ROS_ERROR("Not specified turtle's name as an argument!"); 
  	return -1;
  };
  
  ros::NodeHandle nh;
  
  turtle_name = argv[1];
  ros::Subscriber sub = listen();
  walk(turtle_name);
  follow(turtle_name, rescuer_turtle);
  return 0;
}

