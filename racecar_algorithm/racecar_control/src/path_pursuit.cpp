#include "path_pursuit.h"

#define DEBUG
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
    nodeHandle.param<std::string>("cmd_topic", cmdTopic, "/cmd_ackermann");
    nodeHandle.param<std::string>("path_topic", pathTopic, "/move_base/TebLocalPlannerROS/local_plan");
    nodeHandle.param<std::string>("debug_path_topic", debugPathTopic, "/debug_path_topic");
    
    nodeHandle.param<bool>("enable_publish_debug_path", enablePublishDebugPath, false);

    nodeHandle.param<double>("speed", speedSet, 1.0);
    nodeHandle.param<double>("max_steering_angle", steerAngleMax, 0.785398163);
    nodeHandle.param<double>("wheel_base", wheelBase, 1);

    odomSubscriber.subscribe(nodeHandle, odomTopic, 1);
    //nodeHandle.subscribe(odomTopic, 1, &PathPursuit::OdomDataCallBack, this);
    pathSubscriber = nodeHandle.subscribe<nav_msgs::Path>(pathTopic, 1, &PathPursuit::PathDataCallBack, this);
    cmdPublisher = nodeHandle.advertise<ackermann_msgs::AckermannDriveStamped>(cmdTopic, 1);
    if (enablePublishDebugPath) debugPathPublisher = nodeHandle.advertise<nav_msgs::Path>(debugPathTopic ,1);
}

void PathPursuit::ResetVariables(void) {
    pathFrame = "";
    pathBuffer.clear();
    positionBuffer.clear();
    isReachGoal = false;
    if (odomMsgFilter) odomMsgFilter->clear();
    odomMsgFilter = nullptr;
}

void PathPursuit::OdomDataCallBack(const nav_msgs::OdometryConstPtr & data)
{
    DEBUG_INFO("Received odom data");
    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.drive.speed = 0;
    cmd.drive.steering_angle = 0;
    cmd.drive.steering_angle_velocity = 0;

    if (isReachGoal) {
        ROS_INFO("Goal is reached and way for new goal");
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
        int goalIndex = -1;
        for (auto index = 1; index < positionBuffer.size(); ++index) {
            distances[index] = Position::distanceBetweenPositions(curPosition, positionBuffer[index]);
            if (distances[index] > 0.1 && distances[index] < 0.3) { // 寻找一定距离范围内的点
                if (fabs(Position::angleBetweenPositions(curPosition, positionBuffer[index])) < steerAngleMax) {
                    goalIndex = index; // 如果该点存在,那么将该点设为当前目标,并停止搜索
                    break;
                }
            }
        }
        if (goalIndex == -1) goalIndex = 1; // 如果没有找到任何点,设置目标为最近的那个点

        Position goalPosition = pathBuffer[goalIndex];

        DEBUG_INFO("curPoint info is" + curPosition.toString());
        DEBUG_INFO("goalIndex is " + std::to_string(goalIndex));
        DEBUG_INFO("goalPoint info is" + goalPosition.toString());
        DEBUG_INFO("goalDistance is " + std::to_string(Position::distanceBetweenPositions(curPosition, goalPosition)));
        DEBUG_INFO("goalAngle is " + std::to_string(Position::angleBetweenPositions(curPosition, goalPosition)));
        if (enablePublishDebugPath) {
            nav_msgs::Path debugPath;
            debugPath.header.frame_id = pathFrame;
            debugPath.header.stamp = ros::Time::now();
            debugPath.poses.push_back(curPosition.toPoseStamped());
            debugPath.poses.push_back(goalPosition.toPoseStamped());
            debugPathPublisher.publish(debugPath);
        }

        // Transform the goal point to vehicle coordinates. 
        //double distanceXWorld = goalPosition.x - curPosition.x; // 世界坐标系下的x轴距离
        //double distanceYWorld = goalPosition.y - curPosition.y; 
        //double distanceXRobot = distanceXWorld * cos(curPosition.yaw) + distanceYWorld * sin(curPosition.yaw); // 机器人坐标系下的x轴距离
        //double distanceYRobot = distanceYWorld * cos(curPosition.yaw) - distanceXWorld * sin(curPosition.yaw);

        // Calculate the curvature = 1/r = 2x/l^2
        // The curvature is transformed into steering wheel angle by the vehicle on board controller.
        // Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.

        // 原python程序如下，但我认为公式有误，所以做了修改 by 刘力铭
        // diff_angle = path_points_w[goal] - yaw # Find the turning angle
        // r = L/(2*math.sin(diff_angle)) # Calculate the turning radius
        // angle = 2 * math.atan(0.4/r) # Find the wheel turning radius
        double diffAngle = Position::angleBetweenPositions(curPosition, goalPosition); // Find the turning angle
        double turningRadius = distances[goalIndex] / (2 * sin(diffAngle / 2.0)); // Calculate the turning radius
        double turningAngle = atan(wheelBase / turningRadius); // Find the wheel turning radius

        if (turningAngle > steerAngleMax) turningAngle = steerAngleMax;
        if (turningAngle < -steerAngleMax) turningAngle = -steerAngleMax;

        cmd.drive.speed = speedSet;
        cmd.drive.steering_angle = turningAngle;
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
        DEBUG_INFO("Path Info " + std::to_string(index) + positionBuffer[index].toString());
    }
    
    odomMsgFilter = new tf::MessageFilter<nav_msgs::Odometry>(odomSubscriber, tfListener, pathFrame, 1);
    odomMsgFilter->registerCallback(&PathPursuit::OdomDataCallBack, this);
}