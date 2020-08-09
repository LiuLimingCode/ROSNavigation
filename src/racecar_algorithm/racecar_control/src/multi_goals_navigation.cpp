#include <iostream>
#include <string>
#include "ros/ros.h"
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>


class MultiGoalsNavigation
{
private:

    ros::NodeHandle nodeHandle;
    ros::Subscriber amclSubscriber, goalSubscriber;
    ros::Publisher goalPublisher;

    std::string mapFrame, amclPoseTopic, initPoseTopic, goalTopic;

    double goalRadius, mapOffsetX, mapOffsetY;

    bool debugMode, navigationStarted;

    XmlRpc::XmlRpcValue locationsXmlRpc, goalsIdXmlRpc, goalsStaticXmlRpc;

    std::vector<geometry_msgs::Point> locationVector;
    std::vector<int> goalsIdVector;
    std::vector<bool> goalsStaticVector;

    int currentGoalIndex = 0;
    const geometry_msgs::Point * currentGoal;

public:
    ~MultiGoalsNavigation() = default;
    MultiGoalsNavigation(ros::NodeHandle node)
    {
        nodeHandle = node;
        ros::NodeHandle _n("~");

        _n.param<std::string>("map_frame", mapFrame, "map");
        _n.param<std::string>("amcl_pose_topic", amclPoseTopic, "/amcl_pose");
        _n.param<std::string>("init_pose_topic", initPoseTopic, "/initialpose");
        _n.param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");

        _n.param<double>("goal_radius", goalRadius, 1.0);
        _n.param<double>("map_offset_x", mapOffsetX, 1.0);
        _n.param<double>("map_offset_y", mapOffsetY, 1.0);

        _n.param<bool>("debug_mode", debugMode, false);

        _n.getParam("locations", locationsXmlRpc);
        _n.getParam("goals_id", goalsIdXmlRpc);
        _n.getParam("goals_static", goalsStaticXmlRpc);

        amclSubscriber = nodeHandle.subscribe(amclPoseTopic, 1, &MultiGoalsNavigation::amclPoseCallBack, this);
        goalSubscriber = nodeHandle.subscribe(initPoseTopic, 1, &MultiGoalsNavigation::initPoseoalCallBack, this);
        goalPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>(goalTopic, 10);

        try
        {
            locationVector = getPointVectorFromXMLRPC(locationsXmlRpc, "locations");
            goalsIdVector = getNumberVectorFromXMLRPC<int>(goalsIdXmlRpc, "goals_id");
            goalsStaticVector = getNumberVectorFromXMLRPC<bool>(goalsStaticXmlRpc, "goals_static");
        } 
        catch(const std::exception& ex)
        {
            checkCondition(true, ex.what());
        }

        checkCondition(goalsIdVector.size() != goalsStaticVector.size(), "fatal error: the size of goals_id array param is not equal to the size of goals_static array param");
        checkCondition(goalsIdVector.size() == 0, "fatal error: the size of goals_id array param is 0");

        for(auto index = 0; index < goalsIdVector.size(); ++index)
        {
            checkCondition(goalsIdVector[index] > locationVector.size(), "fatal error: some elements in goals_id array param is large than the size of locations array param");
            if(debugMode) ROS_INFO("index: %d, id: %d, static: %d, goal.x: %lf, goal.y: %lf", index, goalsIdVector[index], (int)goalsStaticVector[index], 
                locationVector[goalsIdVector[index] - 1].x, locationVector[goalsIdVector[index] - 1].y);
        }
    }

    void checkCondition(bool condition, std::string errorStr)
    {
        if(condition)
        {
            ROS_ERROR_STREAM(errorStr);
            ros::shutdown();
        }
    }

    double getYawFromPose(const geometry_msgs::Pose& pose)
    {
        tf::Pose p;
        tf::poseMsgToTF(pose, p);
        const double psi = tf::getYaw(p.getRotation());

        return psi;
    }

    void publishGoal(const geometry_msgs::Point& goal)
    {
        geometry_msgs::PoseStamped pose;
        tf::Quaternion q = tf::createQuaternionFromYaw(0);

        pose.header.frame_id = mapFrame;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = goal.x - mapOffsetX;
        pose.pose.position.y = goal.y - mapOffsetY;
        pose.pose.position.z = 0;
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();

        goalPublisher.publish(pose);
    }

    void amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
    {
        if(!navigationStarted) return;

        double dx = amclMsg->pose.pose.position.x - currentGoal->x;
        double dy = amclMsg->pose.pose.position.y - currentGoal->y;
        double dist = sqrt(dx * dx + dy * dy);

        if(dist < goalRadius)
        {
            if(currentGoalIndex < goalsIdVector.size() - 1)
            {
                currentGoalIndex++;
                currentGoal = &locationVector[goalsIdVector[currentGoalIndex] - 1];
                if(debugMode) ROS_INFO("current goal index: %d, x: %lf, y: %lf", currentGoalIndex, currentGoal->x, currentGoal->y);
                publishGoal(*currentGoal);
            }
            else
            {
                ROS_INFO("all goals reached, multi navigation finished.");
                navigationStarted = false;
            }
        }
    }

    void initPoseoalCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data)
    {
        geometry_msgs::Point initPoint = data->pose.pose.position;
        double initYaw = getYawFromPose(data->pose.pose);
 
        navigationStarted = true;
        ROS_INFO("received init pose msg, started multi navigation");
        if(debugMode) ROS_INFO("init pose x: %lf, y: %lf, yaw: %lf", initPoint.x, initPoint.y, initYaw);

        currentGoalIndex = 0;
        currentGoal = &locationVector[goalsIdVector[currentGoalIndex] - 1];
        if(debugMode) ROS_INFO("current goal index: %d, x: %lf, y: %lf", currentGoalIndex, currentGoal->x, currentGoal->y);
        publishGoal(*currentGoal);
    }

    template<typename T>
    T getNumberFromXMLRPC(XmlRpc::XmlRpcValue& xmlrpc, const std::string& full_param_name)
    {
        // Make sure that the value we're looking at is either a double or an int.
        if (xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeInt &&
            xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
            xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
        {
            std::string& value_string = xmlrpc;
            ROS_FATAL("Values in the (param %s) must be numbers. Found value %s.", full_param_name.c_str(), value_string.c_str());
            throw std::runtime_error("Values in the param must be numbers");
        }
        T number;
        if(xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeInt) number = (int)(xmlrpc);
        else if(xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeDouble) number = (double)(xmlrpc);
        else if(xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeBoolean) number = (bool)(xmlrpc);

        return number;
    }

    std::vector<geometry_msgs::Point> getPointVectorFromXMLRPC(XmlRpc::XmlRpcValue& xmlrpc, const std::string& full_param_name)
    {
        if (xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_FATAL("The para must be specified as list of lists on the parameter server, %s was specified as %s",
                        full_param_name.c_str(), std::string(xmlrpc).c_str());
            throw std::runtime_error("The param must be specified as list of lists, points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
        }

        std::vector<geometry_msgs::Point> pointVector;
        geometry_msgs::Point pt;
        
        for (int i = 0; i < xmlrpc.size(); ++i)
        {
            // Make sure each element of the list is an array of size 2. (x and y coordinates)
            XmlRpc::XmlRpcValue point = xmlrpc[ i ];
            if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                point.size() != 2)
            {
            ROS_FATAL("The (parameter %s) must be specified as list of lists on the parameter server eg: "
                        "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                        full_param_name.c_str());
            throw std::runtime_error("The param must be specified as list of lists on the parameter server eg: "
                                    "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
            }

            pt.x = getNumberFromXMLRPC<double>(point[ 0 ], full_param_name);
            pt.y = getNumberFromXMLRPC<double>(point[ 1 ], full_param_name);

            pointVector.push_back(pt);
        }
        return pointVector;
    }

    template<typename T>
    std::vector<T> getNumberVectorFromXMLRPC(XmlRpc::XmlRpcValue& xmlrpc, const std::string& full_param_name)
    {
        if (xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_FATAL("The para must be specified as list of lists on the parameter server, %s was specified as %s",
                        full_param_name.c_str(), std::string(xmlrpc).c_str());
            throw std::runtime_error("The param must be specified as list of lists, points eg: [x1, x2, ..., xn]");
        }

        std::vector<T> numberVector;
        T nb;

        for (int i = 0; i < xmlrpc.size(); ++i)
        {
            nb = getNumberFromXMLRPC<T>(xmlrpc[ i ], full_param_name);

            numberVector.push_back(nb);
        }
        return numberVector;
    }
};







int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_goals_navgation_node");

    MultiGoalsNavigation controller(ros::NodeHandle("~"));
    
    ros::spin();
    
    return 0;
}
