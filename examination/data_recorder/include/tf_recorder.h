#ifndef TF_RECORDER_H
#define TF_RECORDER_H

#include <ros/ros.h>
#include "base_recorder.h"
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

#define STRING_TF                   (std::string("TF"))

class TFRecorder : public BaseRecorder
{

public:
	TFRecorder(ros::NodeHandle& node, std::string& topicName, std::string& topicType,std::string& topicTitle, std::string& targetFrame, std::string& sourceFrame);
	~TFRecorder() = default;

	std::string printfDataTitle(void);
	std::string printfData(void);

    void DataCallBack(const tf2_msgs::TFMessage::ConstPtr& data);
	
protected:
    std::string targetFrame;
    std::string sourceFrame;
    tf::TransformListener listener;
	tf::StampedTransform dataReceived;
    

};

#endif
