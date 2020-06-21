#include "accel_recorder.h"

namespace DataRecorder {

	template<typename T>
	std::string AccelRecorder<T>::printfDataTitle(void)
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

	template<typename T>
	std::string AccelRecorder<T>::printfData(void)
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

	template<typename T>
	AccelRecorder<T>::AccelRecorder(ros::NodeHandle& node, std::string& topicName,std::string& topicTitle)
	{
		this->topicName = topicName;
		this->topicTitle = topicTitle;

		subscriber = node.subscribe<T>(topicName, 1, &AccelRecorder::DataCallBack, this);

		flagDataReceived = false;
	}

	template<typename T>
	void AccelRecorder<T>::DataCallBack(const boost::shared_ptr<T const> & data)
	{
		flagDataReceived = true;
		const void* dataPtr = data.get();
		
		if(IsSameType<geometry_msgs::Accel, T>::result)
			dataReceived = *(geometry_msgs::Accel *)dataPtr;
		if(IsSameType<geometry_msgs::AccelStamped, T>::result)
			dataReceived = (*(geometry_msgs::AccelStamped *)dataPtr).accel;
		if(IsSameType<geometry_msgs::AccelWithCovariance, T>::result)
			dataReceived = (*(geometry_msgs::AccelWithCovariance *)dataPtr).accel;
		if(IsSameType<geometry_msgs::AccelWithCovarianceStamped, T>::result)
			dataReceived = (*(geometry_msgs::AccelWithCovarianceStamped *)dataPtr).accel.accel;

	}

} // namespace DataRecorder

// 指明模板类只能使用这些类型
template class DataRecorder::AccelRecorder<geometry_msgs::Accel>;
template class DataRecorder::AccelRecorder<geometry_msgs::AccelStamped>;
template class DataRecorder::AccelRecorder<geometry_msgs::AccelWithCovariance>;
template class DataRecorder::AccelRecorder<geometry_msgs::AccelWithCovarianceStamped>;
