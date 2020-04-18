#include "accel_recorder.h"

std::string AccelRecorder::printfDataTitle(void)
{
	std::string str
		= topicTitle + ".linear.x" + "\t"
		+ topicTitle + ".linear.y" + "\t"
		+ topicTitle + ".linear.z" + "\t"
		+ topicTitle + ".angular.x" + "\t"
		+ topicTitle + ".angular.y" + "\t"
		+ topicTitle + ".angular.z" + "\t";

	return(str);
}

std::string AccelRecorder::printfData(void)
{
	std::string str;
	if(flagDataReceived)
	{
		flagDataReceived = false;

		str = std::to_string(dataReceived.linear.x) + "\t"
		    + std::to_string(dataReceived.linear.y) + "\t"
		    + std::to_string(dataReceived.linear.z) + "\t"
		    + std::to_string(dataReceived.angular.x) + "\t"
		    + std::to_string(dataReceived.angular.y) + "\t"
		    + std::to_string(dataReceived.angular.z) + "\t";
	}
	else
	{
		str = std::string("\t") + "\t" + "\t" + "\t" + "\t" + "\t";
	}

	return(str);
}

AccelRecorder::AccelRecorder(ros::NodeHandle& node, std::string& topicName, std::string& topicType,std::string& topicTitle)
{
	this->topicName = topicName;
	this->topicType = topicType;
	this->topicTitle = topicTitle;

	if(topicType == "Accel") subscriber = node.subscribe<geometry_msgs::Accel>(topicName, 1, &AccelRecorder::DataAccelCallBack, this);
	else if(topicType == "AccelStamped") subscriber = node.subscribe<geometry_msgs::AccelStamped>(topicName, 1, &AccelRecorder::DataAccelStampedCallBack, this);
	else if(topicType == "AccelWithCovariance") subscriber = node.subscribe<geometry_msgs::AccelWithCovariance>(topicName, 1, &AccelRecorder::DataAccelWithCovarianceCallBack, this);
	else if(topicType == "AccelWithCovarianceStamped") subscriber = node.subscribe<geometry_msgs::AccelWithCovarianceStamped>(topicName, 1, &AccelRecorder::DataAccelWithCovarianceStampedCallBack, this);

	flagDataReceived = false;
}

void AccelRecorder::DataAccelCallBack(const geometry_msgs::Accel::ConstPtr& data)
{
	flagDataReceived = true;
	dataReceived = *data;
}

void AccelRecorder::DataAccelStampedCallBack(const geometry_msgs::AccelStamped::ConstPtr& data)
{
	flagDataReceived = true;
	dataReceived = data->accel;
}

void AccelRecorder::DataAccelWithCovarianceCallBack(const geometry_msgs::AccelWithCovariance::ConstPtr& data)
{
	flagDataReceived = true;
	dataReceived = data->accel;
}

void AccelRecorder::DataAccelWithCovarianceStampedCallBack(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& data)
{
	flagDataReceived = true;
	dataReceived = data->accel.accel;
}

