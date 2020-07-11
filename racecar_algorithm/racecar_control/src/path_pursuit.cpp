#include "path_pursuit.h"

//#define DEBUG
#ifdef DEBUG
#define DEBUG_INFO ROS_INFO_STREAM
#else
#define DEBUG_INFO
#endif

PathPursuit::PathPursuit(ros::NodeHandle handle)
{
    nodeHandle = handle;
    
    ResetVariables();

    nodeHandle.param<std::string>("odom_topic", odomTopic, "/odom");
    nodeHandle.param<std::string>("path_topic", pathTopic, "/move_base/TebLocalPlannerROS/local_plan");
    nodeHandle.param<std::string>("cmd_topic", cmdTopic, "/cmd_ackermann");

    odomSubscriber.subscribe(nodeHandle, odomTopic, 1);
    //nodeHandle.subscribe(odomTopic, 1, &PathPursuit::OdomDataCallBack, this);
    nodeHandle.subscribe(pathTopic, 1, &PathPursuit::PathDataCallBack, this);
    cmdPublisher = nodeHandle.advertise<ackermann_msgs::AckermannDriveStamped>(cmdTopic, 1);
}

void PathPursuit::ResetVariables(void) {
    pathFrame = "";
    pathBuffer.clear();
    positionBuffer.clear();
    isReachGoal = false;
    odomMsgFilter->clear();
}

void PathPursuit::OdomDataCallBack(const nav_msgs::OdometryConstPtr & data)
{
    DEBUG_INFO("Received odom data");
    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.drive.speed = 0;
    cmd.drive.steering_angle = 0;
    cmd.drive.steering_angle_velocity = 0;

    if (isReachGoal) {
        DEBUG_INFO("Goal is reached and way for new goal");
        ResetVariables();
    } else if (!pathBuffer.empty()) {
        // Read the current pose of the car from particle filter
        std_msgs::Header headerRead = data->header;
        geometry_msgs::Twist twistRead = data->twist.twist;
        geometry_msgs::PoseStamped poseRead;
        poseRead.pose = data->pose.pose;
        poseRead.header = headerRead;
        
        try {
            tfListener.transformPose(pathFrame, poseRead, poseRead);
        } catch(tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            return;
        }

        // Find the path point closest to the vehichle tat is >= 1 lookahead distance from vehicle's current location.
        Position curPosition(poseRead);
        double distances[positionBuffer.size()] = {0};
        double minDistance = Position::distance(curPosition, positionBuffer[0]);
        int minDistanceIndex = 0;
        for (auto index = 1; index < positionBuffer.size(); ++index) {
            distances[index] = Position::distance(curPosition, positionBuffer[index]);
            if (distances[index] < minDistance) {
                minDistance = distances[index];
                minDistanceIndex = index;
            }
        }
    }

    cmdPublisher.publish(cmd);
}

void PathPursuit::PathDataCallBack(const nav_msgs::PathConstPtr & data)
{
    DEBUG_INFO("Received path data");
    ResetVariables();
    pathFrame = data->header.frame_id;
    pathBuffer = data->poses;
    for (auto index = 0; index < pathBuffer.size(); ++index) {
        positionBuffer.push_back(Position(pathBuffer[index]));
    }
    
    odomMsgFilter = new tf::MessageFilter<nav_msgs::Odometry>(odomSubscriber, tfListener, pathFrame, 1);
    odomMsgFilter->registerCallback(&PathPursuit::OdomDataCallBack, this);
}