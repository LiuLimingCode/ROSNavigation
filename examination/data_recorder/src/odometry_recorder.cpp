#include "odometry_recorder.h"

std::string OdometryRecorder::printfDataTitle(void)
{
	std::string str
		= topicTitle + ".twist.twist.linear.x" + "\t"
		+ topicTitle + ".twist.twist.linear.y" + "\t"
		+ topicTitle + ".twist.twist.linear.z" + "\t"
		+ topicTitle + ".twist.twist.angular.x" + "\t"
		+ topicTitle + ".twist.twist.angular.y" + "\t"
		+ topicTitle + ".twist.twist.angular.z" + "\t"
		+ topicTitle + ".pose.pose.position.x" + "\t"
		+ topicTitle + ".pose.pose.position.y" + "\t"
		+ topicTitle + ".pose.pose.position.z" + "\t"
		+ topicTitle + ".pose.pose.orientation.x" + "\t"
		+ topicTitle + ".pose.pose.orientation.y" + "\t"
		+ topicTitle + ".pose.pose.orientation.z" + "\t"
		+ topicTitle + ".pose.pose.orientation.w" + "\t";

	return(str);
}

std::string OdometryRecorder::printfData(void)
{
	std::string str;
	if(flagDataReceived)
	{
		flagDataReceived = false;
		str = std::to_string(dataReceived.twist.twist.linear.x) + "\t"
		    + std::to_string(dataReceived.twist.twist.linear.y) + "\t"
		    + std::to_string(dataReceived.twist.twist.linear.z) + "\t"
		    + std::to_string(dataReceived.twist.twist.angular.x) + "\t"
		    + std::to_string(dataReceived.twist.twist.angular.y) + "\t"
		    + std::to_string(dataReceived.twist.twist.angular.z) + "\t"
		    + std::to_string(dataReceived.pose.pose.position.x) + "\t"
		    + std::to_string(dataReceived.pose.pose.position.y) + "\t"
		    + std::to_string(dataReceived.pose.pose.position.z) + "\t"
		    + std::to_string(dataReceived.pose.pose.orientation.x) + "\t"
		    + std::to_string(dataReceived.pose.pose.orientation.y) + "\t"
		    + std::to_string(dataReceived.pose.pose.orientation.z) + "\t"
		    + std::to_string(dataReceived.pose.pose.orientation.w) + "\t";
	}
	else
	{
		str = std::string("\t") + "\t" + "\t" + "\t" + "\t" + "\t" + "\t" + "\t" + "\t" + "\t" + "\t"+ "\t" + "\t";
	}

	return(str);
}

OdometryRecorder::OdometryRecorder(ros::NodeHandle& node, std::string& topicName, std::string& topicType,std::string& topicTitle)
{
	this->topicName = topicName;
	this->topicType = topicType;
	this->topicTitle = topicTitle;

	subscriber = node.subscribe<nav_msgs::Odometry>(topicName, 1, &OdometryRecorder::DataCallBack, this);
	flagDataReceived = false;
}

void OdometryRecorder::DataCallBack(const nav_msgs::Odometry::ConstPtr& data)
{
	flagDataReceived = true;
	dataReceived = *data;
}

