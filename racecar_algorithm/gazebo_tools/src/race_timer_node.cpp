#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"


bool isGoalReceived = false;
bool use_goal_info = false;

double goal_x = 0.0;
double goal_y = 0.0;

geometry_msgs::Pose goalPose;

ros::Duration cost_time;

std::string goal_topic;
std::string odom_topic;
std::string consider_x;
std::string consider_y;

int min_time;

void goalReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    cost_time = ros::Duration(0);
    goalPose = msg->pose;
    isGoalReceived = true;
    //isGoalReached = false;
}

void odomReceivedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!isGoalReceived) return;
    geometry_msgs::Pose pose = msg->pose.pose;
    geometry_msgs::Pose destination;

    if(use_goal_info) destination = goalPose;
    else destination.position.x = goal_x, destination.position.y = goal_y;

    bool flag_x = false, flag_y = false;
    if(consider_x == "ignore") flag_x = true;
    else if(consider_x == "less")
        if(destination.position.x > pose.position.x) flag_x = true;
    else if(consider_x == "large")
        if(destination.position.x < pose.position.x) flag_x = true;

    if(consider_y == "ignore") flag_y = true;
    else if(consider_y == "less")
        if(destination.position.y > pose.position.y) flag_y = true;
    else if(consider_y == "large")
        if(destination.position.y < pose.position.y) flag_y = true;

    if(cost_time.sec > min_time)
    {
        if(flag_x && flag_y)
        {
            isGoalReceived = false;
            ROS_INFO("goal reached, cost time is : %d.%d", cost_time.sec, cost_time.nsec);
        }
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "race_timer_node");

	ros::NodeHandle node("~");

    node.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
    node.param<std::string>("odom_topic", odom_topic, "/odom");
    node.param<std::string>("consider_x", consider_x, "ignore");
    node.param<std::string>("consider_y", consider_y, "ignore");
    node.param<bool>("use_goal_info", use_goal_info, false);
    node.param<int>("min_time", min_time, 2);
    node.param<double>("goal_x", goal_x, 0.0);
    node.param<double>("goal_y", goal_y, 0.0);

    ros::Subscriber goal_subscriber = node.subscribe("/move_base_simple/goal", 1, goalReceivedCallback);
    ros::Subscriber pose_subscriber = node.subscribe("/odom", 1, odomReceivedCallback);
    
    ros::Duration loop(0.01);

    while(ros::ok())
    {
        if(isGoalReceived)
        {
            cost_time += loop;
            ROS_INFO("cost time : %d.%d", cost_time.sec, cost_time.nsec);
        }
        loop.sleep();
        ros::spinOnce();
    }


    return(0);
}