#include "imu_recorder.h"

namespace DataRecorder {

	std::string ImuRecorder::printfDataTitle(void)
	{
		std::string str
			= topicTitle + ".orientation.x" + "\t"
			+ topicTitle + ".orientation.y" + "\t"
			+ topicTitle + ".orientation.z" + "\t"
			+ topicTitle + ".orientation.w" + "\t"
			+ topicTitle + ".angular_velocity.x" + "\t"
			+ topicTitle + ".angular_velocity.y" + "\t"
			+ topicTitle + ".angular_velocity.z" + "\t"
			+ topicTitle + ".linear_acceleration.x" + "\t"
			+ topicTitle + ".linear_acceleration.y" + "\t"
			+ topicTitle + ".linear_acceleration.z" + "\t";

		return(str);
	}

	std::string ImuRecorder::printfData(void)
	{
		std::string str;
		if(flagDataReceived)
		{
			flagDataReceived = false;
			str = std::to_string(dataReceived.orientation.x) + "\t"
				+ std::to_string(dataReceived.orientation.y) + "\t"
				+ std::to_string(dataReceived.orientation.z) + "\t"
				+ std::to_string(dataReceived.orientation.w) + "\t"
				+ std::to_string(dataReceived.angular_velocity.x) + "\t"
				+ std::to_string(dataReceived.angular_velocity.y) + "\t"
				+ std::to_string(dataReceived.angular_velocity.z) + "\t"
				+ std::to_string(dataReceived.linear_acceleration.x) + "\t"
				+ std::to_string(dataReceived.linear_acceleration.y) + "\t"
				+ std::to_string(dataReceived.linear_acceleration.z) + "\t";
		}
		else
		{
			str = std::string("\t") + "\t" + "\t" + "\t" + "\t" + "\t" + "\t" + "\t" + "\t" + "\t";
		}

		return(str);
	}

	ImuRecorder::ImuRecorder(ros::NodeHandle& node, std::string& topicName,std::string& topicTitle)
	{
		this->topicName = topicName;
		this->topicTitle = topicTitle;

		subscriber = node.subscribe<sensor_msgs::Imu>(topicName, 1, &ImuRecorder::DataCallBack, this);
		flagDataReceived = false;
	}

	void ImuRecorder::DataCallBack(const sensor_msgs::Imu::ConstPtr& data)
	{
		flagDataReceived = true;
		dataReceived = *data;
	}

} // namespace DataRecorder
