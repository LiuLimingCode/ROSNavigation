#ifndef BASE_RECORDER_H
#define BASE_RECORDER_H

#include <ros/ros.h>

namespace DataRecorder {

	class BaseRecorder
	{

	public:
		virtual std::string printfDataTitle(void) = 0;
		virtual std::string printfData(void) = 0;
		bool isDataReceived() const { return(flagDataReceived); }

	protected:
		std::string topicName;
		std::string topicTitle;
		ros::Subscriber subscriber;
		bool flagDataReceived = false;

	};

	template<class T, class U>
	struct IsSameType
	{
		enum {result=false};
	};

	template<class T>
	struct IsSameType<T, T>
	{
		enum {result=true};
	};

} // namespace DataRecorder

#endif
