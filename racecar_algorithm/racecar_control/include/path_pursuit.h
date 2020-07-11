#include <ros/ros.h>
#include <cmath>

#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

struct Position {
    Position(double _x, double _y, double _yaw) :
    x(_x), y(_y), yaw(_yaw) {}

    Position(geometry_msgs::PoseStamped pose) {
        x = pose.pose.position.x;
        y = pose.pose.position.y;

        tf::Quaternion quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w);
        yaw = quaternion.getAxis()[2];
    }

    static double distance(Position p1, Position p2) {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    double x;
    double y;
    double yaw;
};

class PathPursuit
{
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher cmdPublisher;
    message_filters::Subscriber<nav_msgs::Odometry> odomSubscriber;
    tf::TransformListener tfListener;
    tf::MessageFilter<nav_msgs::Odometry> * odomMsgFilter;

    std::string odomTopic;
    std::string pathTopic;
    std::string cmdTopic;
    std::string pathFrame;

    std::vector<geometry_msgs::PoseStamped> pathBuffer;
    std::vector<Position> positionBuffer;

    bool isReachGoal;

public:
    PathPursuit(ros::NodeHandle handle);
    ~PathPursuit()=default;

    void ResetVariables(void);

    void OdomDataCallBack(const nav_msgs::OdometryConstPtr & data);
    void PathDataCallBack(const nav_msgs::PathConstPtr & data);
};