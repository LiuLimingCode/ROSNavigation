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

namespace DataRecorder {

	template<typename T>
	class AccelRecorder : public BaseRecorder
	{

	public:
		AccelRecorder(ros::NodeHandle& node, std::string& topicName,std::string& topicTitle);
		~AccelRecorder() = default;

		std::string printfDataTitle(void);
		std::string printfData(void);

		void DataCallBack(const boost::shared_ptr<T const> & data);

		
	protected:
		geometry_msgs::Accel dataReceived;
	};

} // namespace DataRecorder

#endif
