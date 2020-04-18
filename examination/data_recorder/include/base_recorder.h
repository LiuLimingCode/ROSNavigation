#ifndef BASE_RECORDER_H
#define BASE_RECORDER_H

#include <ros/ros.h>

class BaseRecorder
{

public:
	virtual std::string printfDataTitle(void) = 0;
	virtual std::string printfData(void) = 0;
	bool isDataReceived() const { return(flagDataReceived); }

protected:
	std::string topicName;
	std::string topicType;
	std::string topicTitle;
	ros::Subscriber subscriber;
	bool flagDataReceived = false;

};

#endif
