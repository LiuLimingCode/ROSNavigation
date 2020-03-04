#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

std::string odom_sub_topic;
std::string path_pub_topic;
int odom_buffer_num;
ros::Subscriber odomSubscriber;
ros::Publisher pathPublisher;

void odomCallBack(const nav_msgs::Odometry::ConstPtr& data)
{
	static std::vector<geometry_msgs::PoseStamped> posesStamped;
	ROS_INFO("geted a odometry data");

	geometry_msgs::PoseStamped pose;
	pose.header = data->header;
	pose.pose = data->pose.pose;
	if(posesStamped.size() >= odom_buffer_num) posesStamped.erase(std::begin(posesStamped));
	posesStamped.push_back(pose);
	
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = data->header.frame_id;

	path.poses.resize(posesStamped.size());
	for (unsigned int index = 0; index < posesStamped.size(); index++) {
        	path.poses[index] = posesStamped[index];
	}
	pathPublisher.publish(path);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_topath");

	ros::NodeHandle node("~");

	node.param<std::string>("odom_sub_topic", odom_sub_topic, "/odom");
	node.param<std::string>("path_pub_topic", path_pub_topic, "/path");
	node.param<int>("odom_buffer_num", odom_buffer_num, 300);

	odomSubscriber = node.subscribe(odom_sub_topic, 10, &odomCallBack);
	pathPublisher = node.advertise<nav_msgs::Path>(path_pub_topic, 10);

	ros::spin();
	return(0);
}
