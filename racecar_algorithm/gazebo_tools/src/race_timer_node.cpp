#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

//#define DEBUG
#ifdef DEBUG
#define DEBUG_INFO ROS_ERROR
#else
#define DEBUG_INFO
#endif

class RaceTimer{

protected:

    ros::NodeHandle nodeHandle;
    ros::Subscriber goalSubscriber;
    ros::Subscriber poseSubscriber;
    tf::TransformListener tfListener;
    //message_filters::Subscriber<nav_msgs::Odometry> odomSubscriber;
    //tf::MessageFilter<nav_msgs::Odometry> * odomMsgFilter;

    bool isGoalReceived = false;
    bool useGoalInfo = false;

    double goalX = 0.0;
    double goalY = 0.0;

    geometry_msgs::Pose goalPose;

    std::string goalTopic;
    std::string odomTopic;
    std::string mapFrame;
    std::string considerX;
    std::string considerY;
    int minTime;

public:

    ros::Duration costTime;

    RaceTimer(ros::NodeHandle handle)
    {
        nodeHandle = handle;

        nodeHandle.param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");
        nodeHandle.param<std::string>("odom_topic", odomTopic, "/odom");
        nodeHandle.param<std::string>("map_frame", mapFrame, "map");
        nodeHandle.param<std::string>("consider_x", considerX, "ignore");
        nodeHandle.param<std::string>("consider_y", considerY, "ignore");
        nodeHandle.param<bool>("use_goal_info", useGoalInfo, false);
        nodeHandle.param<int>("min_time", minTime, 2);
        nodeHandle.param<double>("goal_x", goalX, 0.0);
        nodeHandle.param<double>("goal_y", goalY, 0.0);

        goalSubscriber = nodeHandle.subscribe<geometry_msgs::PoseStamped>(goalTopic, 1, &RaceTimer::goalReceivedCallback, this);
        poseSubscriber = nodeHandle.subscribe<nav_msgs::Odometry>(odomTopic, 1, &RaceTimer::odomReceivedCallback, this);

    //    goalSubscriber = nodeHandle.subscribe<geometry_msgs::PoseStamped>(goalTopic, 1, &RaceTimer::goalReceivedCallback, this);
    //    odomSubscriber.subscribe(nodeHandle, odomTopic, 1);
    //    odomMsgFilter = new tf::MessageFilter<nav_msgs::Odometry>(odomSubscriber, tfListener, mapFrame, 1);
    //    odomMsgFilter->setTolerance(ros::Duration(1));
    //    odomMsgFilter->registerCallback(&RaceTimer::odomReceivedCallback, this);
    }

    void increaseCostTime(ros::Duration duration)
    {
        if(isGoalReceived)
        {
            costTime += duration;
            ROS_INFO("cost time : %d.%d", costTime.sec, costTime.nsec);
        }
    }

    void goalReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        DEBUG_INFO("goalReceivedCallback");
        costTime = ros::Duration(0);
        goalPose = msg->pose;
        isGoalReceived = true;
    }

    void odomReceivedCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if(!isGoalReceived) return;

        DEBUG_INFO("odomReceivedCallback");
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header = msg->header;
        poseStamped.pose = msg->pose.pose;
        geometry_msgs::Pose destination;

        tf::StampedTransform transform;
        try
        {
            tfListener.transformPose(mapFrame, poseStamped, poseStamped);
            //tfListener.lookupTransform(msg->header.frame_id, mapFrame, ros::Time(0), transform);
        }
        catch(tf::TransformException &ex)
        {
            DEBUG_INFO("%s",ex.what());
            return;
        }
        geometry_msgs::Pose pose = poseStamped.pose;
        
        if(useGoalInfo) destination = goalPose;
        else destination.position.x = goalX, destination.position.y = goalY;

        bool flag_x = false, flag_y = false;

        if(considerX == "ignore") flag_x = true;
        else if(considerX == "less") {
            if(pose.position.x < destination.position.x) flag_x = true;
        }    
        else if(considerX == "large"){
            if(pose.position.x > destination.position.x) flag_x = true;
        }     
        else if(considerX == "close") {
            if(fabs(pose.position.x - destination.position.x) < 0.1) flag_x = true;
        }
        
        DEBUG_INFO("pose_x: %lf, destination_x: %lf", pose.position.x, destination.position.x);

        if(considerY == "ignore") flag_y = true;
        else if(considerY == "less") {
            if(pose.position.y < destination.position.y) flag_y = true;
        }
        else if(considerY == "large") {
            if(pose.position.y > destination.position.y) flag_y = true;
        }
        else if(considerY == "close") {
            if(fabs(pose.position.y - destination.position.y) < 0.1) flag_y = true;
        }   

        DEBUG_INFO("pose_y: %lf, destination_y: %lf", pose.position.y, destination.position.y);
        DEBUG_INFO("flag_x: %d, flag_y: %d", (int)flag_x, (int)flag_y);

        if(costTime.sec >= minTime)
        {
            if(flag_x && flag_y)
            {
                isGoalReceived = false;
                ROS_INFO("goal reached, cost time is : %d.%d", costTime.sec, costTime.nsec);
            }
        }
    }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "race_timer_node");

	RaceTimer raceTimer(ros::NodeHandle("~"));
    
    ros::Duration loop(0.01);

    while(ros::ok())
    {
        raceTimer.increaseCostTime(loop);
        loop.sleep();
        ros::spinOnce();
    }

    return(0);
}