#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <visualization_msgs/Marker.h>
#include <string> 

std::string turtle_name;
ros::Publisher marker_pub;
visualization_msgs::Marker points;

void initVisualization(ros::NodeHandle &nh) 
{
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	points.header.frame_id = turtle_name;
    points.header.stamp = ros::Time::now();
    points.ns = turtle_name + "_namespace";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    
    int s = turtle_name.size();
    int id = turtle_name[s - 1] - '0';
    points.id = id;
    points.type = visualization_msgs::Marker::POINTS;
     
    points.scale.x = 0.2;
    points.scale.y = 0.2;
     
    if (id == 1) points.color.g = 1.0f;
    else if (id == 2) points.color.r = 1.0f;
    else points.color.b = 1.0f;
  	points.color.a = 1.0;
}

void poseCallback(const turtlesim::PoseConstPtr& msg) 
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
  
  
  geometry_msgs::Point p;
  p.x = msg->x;
  p.y = msg->y;
  p.z = 0.0;
  points.points.push_back(p);
  marker_pub.publish(points);
}

/* run rviz command:
rosrun rviz rviz -d `rospack find turtle_tf`/rviz/turtle_rviz.rviz
*/
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "my_turtle_broadcaster");
  if (argc != 2) 
  {
  	ROS_ERROR("Not specified turtle's name as an argument!"); 
  	return -1;
  };
  
  turtle_name = argv[1];

  ros::NodeHandle nh;
  initVisualization(nh);
  ros::Subscriber sub = nh.subscribe(turtle_name + "/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

