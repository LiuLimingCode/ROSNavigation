#ifndef ACCEL_RECORDER_H
#define ACCEL_RECORDER_H

#include <ros/ros.h>
#include "base_recorder.h"
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>

#define STRING_ACCEL                      (std::string("Accel"))
#define STRING_ACCELSTAMPED               (std::string("AccelStamped"))
#define STRING_ACCELWITHCOVARIANCE        (std::string("AccelWithCovariance"))
#define STRING_ACCELWITHCOVARIANCESTAMPED (std::string("AccelWithCovarianceStamped"))

class AccelRecorder : public BaseRecorder
{

public:
	AccelRecorder(ros::NodeHandle& node, std::string& topicName, std::string& topicType,std::string& topicTitle);
	~AccelRecorder() = default;

	std::string printfDataTitle(void);
	std::string printfData(void);

	void DataAccelCallBack(const geometry_msgs::Accel::ConstPtr& data);
	void DataAccelStampedCallBack(const geometry_msgs::AccelStamped::ConstPtr& data);
	void DataAccelWithCovarianceCallBack(const geometry_msgs::AccelWithCovariance::ConstPtr& data);
	void DataAccelWithCovarianceStampedCallBack(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& data);

	
protected:
	geometry_msgs::Accel dataReceived;
};

#endif
