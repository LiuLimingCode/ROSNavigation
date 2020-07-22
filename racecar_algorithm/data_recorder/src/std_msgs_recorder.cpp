#include "std_msgs_recorder.h"

namespace DataRecorder {

	template <typename T>
	struct ToString
	{
		static std::string result(T value)
		{
			return std::to_string(value.data);
		}
	};
	template <>
	struct ToString<std_msgs::String>
	{
		static std::string result(std_msgs::String value)
		{
			return value.data;
		}
	};
	template <>
	struct ToString<std_msgs::Duration>
	{
		static std::string result(std_msgs::Duration value)
		{
			double time = value.data.sec + (double)value.data.nsec / (double)1e9;
			return std::to_string(time);
		}
	};
	template <>
	struct ToString<std_msgs::Time>
	{
		static std::string result(std_msgs::Time value)
		{
			double time = value.data.sec + (double)value.data.nsec / (double)1e9;
			return std::to_string(time);
		}
	};

	template<typename T>
	std::string StdMsgsRecorder<T>::printfDataTitle(void)
	{
		std::string str
			= topicTitle + ".data" + "\t";

		return(str);
	}

	template<typename T>
	std::string StdMsgsRecorder<T>::printfData(void)
	{
		std::string str;
		if(flagDataReceived)
		{
			flagDataReceived = false;

			str = ToString<T>::result(dataReceived) + "\t";
		}
		else
		{
			str = std::string("\t");
		}

		return(str);
	}

	template<typename T>
	StdMsgsRecorder<T>::StdMsgsRecorder(ros::NodeHandle& node, std::string& topicName,std::string& topicTitle)
	{
		this->topicName = topicName;
		this->topicTitle = topicTitle;

		subscriber = node.subscribe<T>(topicName, 1, &StdMsgsRecorder::DataCallBack, this);

		flagDataReceived = false;
	}

	template<typename T>
	void StdMsgsRecorder<T>::DataCallBack(const boost::shared_ptr<T const> & data)
	{
		flagDataReceived = true;
		
		dataReceived = *data;

	}

} // namespace DataRecorder

// 指明模板类只能使用这些类型
template class DataRecorder::StdMsgsRecorder<std_msgs::Bool>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Byte>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Char>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Duration>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Float32>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Float64>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Int16>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Int32>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Int64>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Int8>;
template class DataRecorder::StdMsgsRecorder<std_msgs::String>;
template class DataRecorder::StdMsgsRecorder<std_msgs::Time>;
template class DataRecorder::StdMsgsRecorder<std_msgs::UInt16>;
template class DataRecorder::StdMsgsRecorder<std_msgs::UInt32>;
template class DataRecorder::StdMsgsRecorder<std_msgs::UInt64>;
template class DataRecorder::StdMsgsRecorder<std_msgs::UInt8>;
