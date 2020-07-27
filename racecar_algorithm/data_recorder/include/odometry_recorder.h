#ifndef ODOMETRY_RECORDER_H
#define ODOMETRY_RECORDER_H

#include <ros/ros.h>
#include "base_recorder.h"
#include <nav_msgs/Odometry.h>

#define STRING_ODOMETRY                   (std::string("Odometry"))

namespace DataRecorder {

	class OdometryRecorder : public BaseRecorder
	{

	public:
		OdometryRecorder(ros::NodeHandle& node, std::string& topicName,std::string& topicTitle);
		~OdometryRecorder() = default;

		std::string printfDataTitle(void);
		std::string printfData(void);

		void DataCallBack(const nav_msgs::Odometry::ConstPtr& data);
		
	protected:
		nav_msgs::Odometry dataReceived;

	};

} // namespace DataRecorder

#endif
