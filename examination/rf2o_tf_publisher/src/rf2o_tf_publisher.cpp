#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

nav_msgs::Odometry odom_real;
nav_msgs::Odometry odom_rf2o;
std::string odom_real_sub_topic;
std::string odom_rf2o_sub_topic;
std::string odom_real_pub_topic;
std::string odom_rf2o_pub_topic;
std::string odom_real_sub_frame;
std::string odom_rf2o_pub_frame;
bool isRealDataGeted = false, isRf2oDataGeted = false;
ros::Subscriber odomRealSubscriber;
ros::Subscriber odomRf2oSubscriber;
ros::Publisher odomRealPublisher;
ros::Publisher odomRf2oPublisher;

void publishTransformSwitch(void)
{
	if(!(isRealDataGeted && isRf2oDataGeted)) return;

	static tf::TransformBroadcaster odom_broadcaster;

	tf::Transform transformOdomReal(tf::Quaternion(odom_real.pose.pose.orientation.x, odom_real.pose.pose.orientation.y, odom_real.pose.pose.orientation.z, odom_real.pose.pose.orientation.w), tf::Vector3(odom_real.pose.pose.position.x, odom_real.pose.pose.position.y, odom_real.pose.pose.position.z));

	tf::Transform transformOdomRf2o(tf::Quaternion(odom_rf2o.pose.pose.orientation.x, odom_rf2o.pose.pose.orientation.y, odom_rf2o.pose.pose.orientation.z, odom_rf2o.pose.pose.orientation.w), tf::Vector3(odom_rf2o.pose.pose.position.x, odom_rf2o.pose.pose.position.y, odom_rf2o.pose.pose.position.z));

	
	tf::Transform transformSwitch = transformOdomRf2o * transformOdomReal.inverse();
	odom_broadcaster.sendTransform(tf::StampedTransform(transformSwitch, ros::Time::now(), odom_rf2o_pub_frame, odom_real_sub_frame));
	ROS_INFO("send transform");
}

void odomRealCallBack(const nav_msgs::Odometry::ConstPtr& data)
{
	ROS_INFO("receive real odometry");
	isRealDataGeted = true;
	odom_real = *data;
	//publishTransformSwitch();
	nav_msgs::Odometry odom = odom_real;
	odom.header.frame_id = odom_rf2o_pub_frame;
	odomRealPublisher.publish(odom);
}

void odomRf2oCallBack(const nav_msgs::Odometry::ConstPtr& data)
{
	ROS_INFO("receive rf2o odometry");
	isRf2oDataGeted = true;
	odom_rf2o = *data;
	publishTransformSwitch();
	nav_msgs::Odometry odom = odom_rf2o;
	odom.header.frame_id = odom_rf2o_pub_frame;
	odomRf2oPublisher.publish(odom);

	/*static tf::TransformListener listener;
	tf::StampedTransform transform;

	try
	{
		listener.waitForTransform(odom_rf2o_pub_frame, odom.child_frame_id, ros::Time(0), ros::Duration(3.0));
		listener.lookupTransform(odom_rf2o_pub_frame, odom.child_frame_id, ros::Time(0), transform);
		ROS_INFO_STREAM("transform-origin:\n" << "x:" << transform.getOrigin()[0] << "y:" << transform.getOrigin()[1] << "z:" << transform.getOrigin()[2] << "\n" << 
			        "transform-rotation:\n" << "x:" << transform.getRotation().x() << "y:" << transform.getRotation().y() << "z:" << transform.getRotation().z() << "w:" << transform.getRotation().w() << "\n" << 
			        "odom-origin:\n" << "x:" << odom.pose.pose.position.x << "y:" << odom.pose.pose.position.y << "z:" << odom.pose.pose.position.z << "\n" << 
			        "odom-origin:\n" << "x:" << odom.pose.pose.orientation.x << "y:" << odom.pose.pose.orientation.y << "z:" << odom.pose.pose.orientation.z << "w:" << odom.pose.pose.orientation.w);
	}
	catch (tf::TransformException &ex) 
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}*/
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rf2o_tf_publisher");

	ros::NodeHandle node("~");

	node.param<std::string>("odom_real_sub_topic", odom_real_sub_topic, "/odom");
	node.param<std::string>("odom_rf2o_sub_topic", odom_rf2o_sub_topic, "/rf2o_laser_odometry");
	node.param<std::string>("odom_real_pub_topic", odom_real_pub_topic, "/odom_real");
	node.param<std::string>("odom_rf2o_pub_topic", odom_rf2o_pub_topic, "/odom_rf2o");
	node.param<std::string>("odom_real_sub_frame", odom_real_sub_frame, "odom");
	node.param<std::string>("odom_rf2o_pub_frame", odom_rf2o_pub_frame, "odom_rf2o");

	odomRealSubscriber = node.subscribe(odom_real_sub_topic, 10, &odomRealCallBack);
	odomRf2oSubscriber = node.subscribe(odom_rf2o_sub_topic, 10, &odomRf2oCallBack);
	odomRealPublisher = node.advertise<nav_msgs::Odometry>(odom_real_pub_topic, 5);
	odomRf2oPublisher = node.advertise<nav_msgs::Odometry>(odom_rf2o_pub_topic, 5);

	ROS_INFO_STREAM("subscribe real odometry in " << odom_real_sub_topic);
	ROS_INFO_STREAM("subscribe rf2o odometry in " << odom_rf2o_sub_topic);

	ros::spin();
	return(0);
}
