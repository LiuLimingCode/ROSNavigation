#include "tf_recorder.h"

std::string TFRecorder::printfDataTitle(void)
{
	std::string str
		= topicTitle + ".transform.translation.x" + "\t"
		+ topicTitle + ".transform.translation.y" + "\t"
		+ topicTitle + ".transform.translation.z" + "\t"
		+ topicTitle + ".transform.rotation.x" + "\t"
		+ topicTitle + ".transform.rotation.y" + "\t"
		+ topicTitle + ".transform.rotation.z" + "\t"
		+ topicTitle + ".transform.rotation.w" + "\t";

	return(str);
}

std::string TFRecorder::printfData(void)
{
	std::string str;
	if(flagDataReceived)
	{
		flagDataReceived = false;
		str = std::to_string(dataReceived.getOrigin()[0]) + "\t"
		    + std::to_string(dataReceived.getOrigin()[1]) + "\t"
		    + std::to_string(dataReceived.getOrigin()[2]) + "\t"
		    + std::to_string(dataReceived.getRotation().x()) + "\t"
		    + std::to_string(dataReceived.getRotation().y()) + "\t"
		    + std::to_string(dataReceived.getRotation().z()) + "\t"
		    + std::to_string(dataReceived.getRotation().w()) + "\t";
	}
	else
	{
		str = std::string("\t") + "\t" + "\t" + "\t" + "\t"+ "\t" + "\t";
	}

	return(str);
}

TFRecorder::TFRecorder(ros::NodeHandle& node, std::string& topicName, std::string& topicType,std::string& topicTitle, std::string& targetFrame, std::string& sourceFrame)
{
	this->topicName = topicName;
	this->topicType = topicType;
	this->topicTitle = topicTitle;
    this->targetFrame = targetFrame;
    this->sourceFrame = sourceFrame;

    subscriber = node.subscribe<tf2_msgs::TFMessage>(topicName, 1, &TFRecorder::DataCallBack, this);
	flagDataReceived = false;
}

void TFRecorder::DataCallBack(const tf2_msgs::TFMessage::ConstPtr& data)
{
    static tf::StampedTransform lastData;
	try {
        listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), dataReceived);
        if(!(fabs(dataReceived.getOrigin()[0] - lastData.getOrigin()[0]) < 0.000001 &&
        fabs(dataReceived.getOrigin()[1] - lastData.getOrigin()[1]) < 0.000001 &&
        fabs(dataReceived.getOrigin()[2] - lastData.getOrigin()[2]) < 0.000001 &&
        fabs(dataReceived.getOrigin()[3] - lastData.getOrigin()[3]) < 0.000001 &&
        fabs(dataReceived.getRotation().x() - lastData.getRotation().x()) < 0.000001 &&
        fabs(dataReceived.getRotation().y() - lastData.getRotation().y()) < 0.000001 &&
        fabs(dataReceived.getRotation().z() - lastData.getRotation().z()) < 0.000001 &&
        fabs(dataReceived.getRotation().w() - lastData.getRotation().w()) < 0.000001))
        {
            flagDataReceived = true;
            lastData = dataReceived;
        }

    } catch(tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
}
