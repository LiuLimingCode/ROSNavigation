#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "base_recorder.h"
#include "tf_recorder.h"
#include "imu_recorder.h"
#include "accel_recorder.h"
#include "odometry_recorder.h"

#define RECORDTOPIC  (std::string("record_topic"))
#define RECORDTPYE   (std::string("record_type"))
#define RECORDTITLE  (std::string("record_title"))
#define TARGETFRAME  (std::string("target_frame"))
#define SOURCEFRAME  (std::string("source_frame"))

std::vector<BaseRecorder*> recorderList;
std::string filePath;
std::ofstream recordFile;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_recorder_node");

	ros::NodeHandle node("~");

	//read params and subscribe to topics
	node.param<std::string>("save_path", filePath, "dataRecorded.txt");
	recorderList.clear();
	for(int index = 0; node.hasParam(RECORDTOPIC + std::to_string(index)) && node.hasParam(RECORDTPYE + std::to_string(index)); ++index)
	{
		std::string recordTopic;
		std::string recordType;
		std::string recordTitle;
		node.getParam(RECORDTOPIC + std::to_string(index), recordTopic);
		node.getParam(RECORDTPYE + std::to_string(index), recordType);
		node.param<std::string>(RECORDTITLE + std::to_string(index), recordTitle, std::to_string(index));
		ROS_INFO_STREAM("subscribed to " << recordTopic << ",data type is " << recordType << ",data title is " << recordTitle);
		if(recordType == STRING_ODOMETRY)
		{
			recorderList.push_back(new OdometryRecorder(node, recordTopic, recordType, recordTitle));
		}
		else if(recordType == STRING_IMU)
		{
			recorderList.push_back(new ImuRecorder(node, recordTopic, recordType, recordTitle));
		}
		else if(recordType == STRING_ACCEL || recordType == STRING_ACCELSTAMPED || recordType == STRING_ACCELWITHCOVARIANCE || recordType == STRING_ACCELWITHCOVARIANCESTAMPED)
		{
			recorderList.push_back(new AccelRecorder(node, recordTopic, recordType, recordTitle));
		}
		else if(recordType == STRING_TF)
		{
			if(node.hasParam(TARGETFRAME + std::to_string(index)) && node.hasParam(SOURCEFRAME + std::to_string(index)))
			{
				std::string targetFrame;
				std::string sourceFrame;
				node.getParam(TARGETFRAME + std::to_string(index), targetFrame);
				node.getParam(SOURCEFRAME + std::to_string(index), sourceFrame);
				recorderList.push_back(new TFRecorder(node, recordTopic, recordType, recordTitle, targetFrame, sourceFrame));
			}
			else
			{
				ROS_ERROR_STREAM("the TF recorder didn't have target_frame or source_frame param!");
			}
		}
	}

	ROS_INFO_STREAM("subscribe to " << recorderList.size() << " topics");
	if(recorderList.size() > 0)
	{
		//open file and print title
		recordFile.open(filePath, std::ios::trunc);
		if(recordFile.fail())
		{
			ROS_ERROR_STREAM("open " << filePath << "failed!");
			return(0);
		}
		ROS_INFO_STREAM("open " << filePath);
		std::string dataTitleStr;
		for(int index = 0; index < recorderList.size(); ++index)
		{
			dataTitleStr += recorderList[index]->printfDataTitle();
		}
		recordFile << "time" << "\t" << dataTitleStr << "\n";

		//print data
		ros::Rate loopRate(1000);
		long loopTimes = 0;
		while(ros::ok())
		{
			ros::spinOnce();
			std::string dataStr;

			bool flagPrintData = false;
			for(int index = 0; index < recorderList.size(); ++index)
			{
				if(recorderList[index]->isDataReceived())
				{
					flagPrintData = true;
					break;
				}
			}

			if(flagPrintData)
			{
				for(int index = 0; index < recorderList.size(); ++index)
				{
					dataStr += recorderList[index]->printfData();
				}
				recordFile << ros::Time::now() << "\t" << dataStr << "\n";
				ROS_INFO_STREAM("looped " << ++loopTimes << " times");
			}

			loopRate.sleep();
		}
	}

	return(0);
}