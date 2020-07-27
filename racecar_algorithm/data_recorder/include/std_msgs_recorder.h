#ifndef STD_MSGS_RECORDER_H
#define STD_MSGS_RECORDER_H

#include <ros/ros.h>
#include "base_recorder.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>

#define STRING_BOOL      (std::string("Bool"))
#define STRING_BYTE      (std::string("Byte"))
#define STRING_CHAR      (std::string("Char"))
#define STRING_DURATION  (std::string("Duration"))
#define STRING_FLOAT32   (std::string("Float32"))
#define STRING_FLOAT64   (std::string("Float64"))
#define STRING_INT16     (std::string("Int16"))
#define STRING_INT32     (std::string("Int32"))
#define STRING_INT64     (std::string("Int64"))
#define STRING_INT8      (std::string("Int8"))
#define STRING_STRING    (std::string("String"))
#define STRING_TIME      (std::string("Time"))
#define STRING_UINT16    (std::string("UInt16"))
#define STRING_UINT32    (std::string("UInt32"))
#define STRING_UINT64    (std::string("UInt64"))
#define STRING_UINT8     (std::string("UInt8"))

namespace DataRecorder {

	template<typename T>
	class StdMsgsRecorder : public BaseRecorder
	{

	public:
		StdMsgsRecorder(ros::NodeHandle& node, std::string& topicName,std::string& topicTitle);
		~StdMsgsRecorder() = default;

		std::string printfDataTitle(void);
		std::string printfData(void);

		void DataCallBack(const boost::shared_ptr<T const> & data);

		
	protected:
		T dataReceived;
	};

} // namespace DataRecorder

#endif
