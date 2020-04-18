#ifndef IMU_RECORDER_H
#define IMU_RECORDER_H

#include <ros/ros.h>
#include "base_recorder.h"
#include <sensor_msgs/Imu.h>

#define STRING_IMU                        (std::string("Imu"))

class ImuRecorder : public BaseRecorder
{

public:
	ImuRecorder(ros::NodeHandle& node, std::string& topicName, std::string& topicType,std::string& topicTitle);
	~ImuRecorder() = default;

	std::string printfDataTitle(void);
	std::string printfData(void);

	void DataCallBack(const sensor_msgs::Imu::ConstPtr& data);
	
protected:
	sensor_msgs::Imu dataReceived;

};

#endif
