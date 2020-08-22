/*
# Copyright 2018 HyphaROS Workshop.
# Latest Modifier: HaoChih, LIN (hypha.ros@gmail.com)
# Original Author: ChanYuan KUO & YoRu LU
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <iostream>
#include "ros/ros.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

//#define DEBUG
#ifdef DEBUG
#define PURE_PURSUIT_INFO ROS_ERROR
#else
#define PURE_PURSUIT_INFO
#endif
#define PI 3.1415926535898

/********************/
/* CLASS DEFINITION */
/********************/
class PurePursuit
{
private:
    ros::NodeHandle n_;
    ros::Subscriber odom_sub, path_sub, goal_sub, amcl_sub, costmap_sub, speed_factor_sub;
    ros::Publisher ackermann_pub, cmdvel_pub, marker_pub, pursuit_path_pub;
    ros::ServiceServer stop_robot_srv;
    ros::Timer timer1, timer2;
    tf::TransformListener tf_listener;

    visualization_msgs::Marker lines, points, goal_circle;
    geometry_msgs::Point odom_goal_pos, goal_pos;
    geometry_msgs::Twist cmd_vel;
    ackermann_msgs::AckermannDriveStamped ackermann_cmd;
    nav_msgs::Odometry odom;
    nav_msgs::Path map_path, odom_path;
    boost::shared_ptr<const nav_msgs::OccupancyGrid> costmap;

    double L, Lfw, Vcmd_max, Vcmd_min, Vcmd_stop, steering_max, lfw, steering, velocity, cost_max, predicted_dist, safe_distance_x, safe_distance_y, slowdown_dist_stop, slowdown_dist_min, slowdown_dist_max, stop_interval, speed_factor = 1.0;
    double steering_gain, base_angle, goal_radius, speed_incremental, speed_expected;
    int controller_freq;
    bool cmd_vel_mode, debug_mode, smooth_accel, stop_robot, enbale_safe_distance;
    bool foundForwardPt, foundPredictedCarPt, goal_received, goal_reached, transform_path_data;
    ros::Time stopTime;


public:

    PurePursuit(void)
    {
        //Private parameters handler
        ros::NodeHandle pn("~");

        //Car parameter
        pn.param("L", L, 0.26); // length of car
        pn.param("Vcmd_max", Vcmd_max, 1.0);// reference speed (m/s)
        pn.param("Vcmd_min", Vcmd_min, 1.0);// reference speed (m/s)
        pn.param("Vcmd_stop", Vcmd_stop, 0.0);
        pn.param("steering_max", steering_max, 1.0);
        pn.param("Lfw", Lfw, 3.0); // forward look ahead distance (m)
        pn.param("predicted_dist", predicted_dist, 0.1);
        pn.param("lfw", lfw, 0.13); // distance between front the center of car

        //Controller parameter
        pn.param("controller_freq", controller_freq, 20);
        pn.param("steering_gain", steering_gain, 1.0); // 在算法计算每一次舵机转角时都会乘上这个值
        pn.param("goal_radius", goal_radius, 0.5); // goal radius (m) 指机器人最后与终点相距多少米,可以判断为机器人到达终点
        pn.param("base_angle", base_angle, 0.0); // neutral point of servo (rad) 在算法计算每一次舵机转角时都会加上这个值
        pn.param("cmd_vel_mode", cmd_vel_mode, false); // whether or not publishing cmd_vel
        pn.param("debug_mode", debug_mode, false); // debug mode
        pn.param("smooth_accel", smooth_accel, true); // smooth the acceleration of car 限制加速度
        pn.param("speed_incremental", speed_incremental, 0.5); // speed incremental value (discrete acceleraton), unit: m/s 机器人加速度,该值乘上 controller_freq 才代表每秒的最大加速度
        pn.param("stop_robot", stop_robot, false);
        pn.param("cost_max", cost_max, 0.25); // distance between front the center of car
        pn.param("slowdown_dist_stop", slowdown_dist_stop, 0.2);
        pn.param("slowdown_dist_min", slowdown_dist_min, 0.3);
        pn.param("slowdown_dist_max", slowdown_dist_max, 1.0);
        pn.param("stop_interval", stop_interval, 0.1);
        pn.param("enable_safe_distance", enbale_safe_distance, true);
        pn.param("safe_distance_x", safe_distance_x, 0.1);
        pn.param("safe_distance_y", safe_distance_y, 0.1);

        //Publishers and Subscribers
        odom_sub = n_.subscribe("/pure_pursuit/odom", 1, &PurePursuit::odomCB, this);
        path_sub = n_.subscribe("/pure_pursuit/global_planner", 1, &PurePursuit::pathCB, this);
        goal_sub = n_.subscribe("/pure_pursuit/goal", 1, &PurePursuit::goalCB, this);
        amcl_sub = n_.subscribe("/amcl_pose", 5, &PurePursuit::amclCB, this);
        costmap_sub = n_.subscribe("/move_base/local_costmap/costmap", 1, &PurePursuit::costmapCB, this);
        speed_factor_sub = n_.subscribe("/multi_goals_navigation_node/speed_factor", 1, &PurePursuit::speedFactorCB, this);
        marker_pub = n_.advertise<visualization_msgs::Marker>("/pure_pursuit/path_marker", 10);
        ackermann_pub = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/pure_pursuit/ackermann_cmd", 1);
        if(cmd_vel_mode) cmdvel_pub = n_.advertise<geometry_msgs::Twist>("/pure_pursuit/cmd_vel", 1);
        pursuit_path_pub = n_.advertise<nav_msgs::Path>("pursuit_path", 1);
        stop_robot_srv = n_.advertiseService("/pure_pursuit/stop_robot", &PurePursuit::stopRobotCB, this);

        //Timer
        timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &PurePursuit::controlLoopCB, this); // Duration(0.05) -> 20Hz


        //Init variables
        foundForwardPt = false;
        goal_received = false;
        goal_reached = false;
        velocity = 0.0;
        steering = base_angle;

        //Show info
        ROS_INFO("[param] base_angle: %f", base_angle);
        ROS_INFO("[param] Vcmd_max: %f", Vcmd_max);
        ROS_INFO("[param] Vcmd_min: %f", Vcmd_min);
        ROS_INFO("[param] Lfw: %f", Lfw);

        //Visualization Marker Settings
        initMarker();

        cmd_vel = geometry_msgs::Twist();
        ackermann_cmd = ackermann_msgs::AckermannDriveStamped();
    }


    // 初始化 visualization_msgs::Marker points, goal_circle;
    void initMarker()
    {
        points.header.frame_id = lines.header.frame_id = goal_circle.header.frame_id = "odom";
        points.ns = lines.ns = goal_circle.ns = "Markers";
        points.action = lines.action = goal_circle.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = lines.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
        points.id = 0;
        lines.id = 1;
        goal_circle.id = 2;

        points.type = visualization_msgs::Marker::POINTS;
        lines.type = visualization_msgs::Marker::LINE_STRIP;
        goal_circle.type = visualization_msgs::Marker::CYLINDER;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        lines.scale.x = 0.05;

        goal_circle.scale.x = goal_radius;
        goal_circle.scale.y = goal_radius;
        goal_circle.scale.z = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        lines.color.b = 1.0;
        lines.color.a = 1.0;

        //goal_circle is yellow
        goal_circle.color.r = 1.0;
        goal_circle.color.g = 1.0;
        goal_circle.color.b = 0.0;
        goal_circle.color.a = 0.5;
    }

    // 得到 odom 数据
    void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        this->odom = *odomMsg;
    }

    // 得到路径数据
    void pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
    {
        this->map_path = *pathMsg;
        transform_path_data = true;
    }

    // 得到 goal 数据,并且转换到 map 坐标系下
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
    {
        this->goal_pos = goalMsg->pose.position;    
        try
        {
            geometry_msgs::PoseStamped odom_goal;
            tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal); // 将goal从map坐标系转换到odom坐标系下
            odom_goal_pos = odom_goal.pose.position;
            goal_received = true;
            goal_reached = false;

            /*Draw Goal on RVIZ*/
            goal_circle.pose = odom_goal.pose;
            marker_pub.publish(goal_circle);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    double getYawFromPose(const geometry_msgs::Pose& carPose)
    {
        tf::Pose pose;
        tf::poseMsgToTF(carPose, pose);
        const double psi = tf::getYaw(pose.getRotation());

        return psi;
    }

    // 判断 wayPt 是否在 carPose 的前面
    bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
    {
        float car2wayPt_x = wayPt.x - carPose.position.x;
        float car2wayPt_y = wayPt.y - carPose.position.y;
        double car_theta = getYawFromPose(carPose);

        float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y; // 将目标点与当前机器人的坐标值转换到机器人坐标系的X轴下,来判断目标点是否在机器人前方
        float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

        if(car_car2wayPt_x > 0) /*is Forward WayPt*/
            return true;
        else
            return false;
    }

    double getDistanceBetweenPoints(const geometry_msgs::Point& toPoint, const geometry_msgs::Point& fromPoint)
    {
        double dx = toPoint.x - fromPoint.x;
        double dy = toPoint.y - fromPoint.y;
        return sqrt(dx * dx + dy * dy);
    }

    // 将path上的所有数据都转移到odom坐标系下
    void transformPathData(void)
    {
        if(map_path.poses.size() == 0) return;
        nav_msgs::Path path;
        if(!goal_reached && transform_path_data)
        {
            transform_path_data = false;
            path.poses.clear();
            for(int i = 0; i< map_path.poses.size(); i++)
            {
                geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
                geometry_msgs::PoseStamped odom_path_pose;

                try
                {
                    tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose); // 将点从 map 坐标系转换到 odom 坐标系下
                    path.poses.push_back(odom_path_pose);
                }
                catch(tf::TransformException &ex)
                {
                    ROS_ERROR("%s",ex.what());
                    return;
                }
            }

            odom_path.poses.clear();
            for(int index = 1; index < path.poses.size(); ++index)
            {
                if(isForwardWayPt(path.poses[index].pose.position, path.poses[index - 1].pose))
                {
                    if(odom_path.poses.empty()) odom_path.poses.push_back(path.poses[index - 1]);
                    odom_path.poses.push_back(path.poses[index]);
                }
            }
            odom_path.header.frame_id = "odom";
            odom_path.header.stamp = ros::Time::now();
            pursuit_path_pub.publish(odom_path);
        }

        if(debug_mode) // 输出路径信息
        {
            PURE_PURSUIT_INFO("path info:");
            for(int index = 0; index < odom_path.poses.size(); ++index)
            {
                PURE_PURSUIT_INFO("index: %d, x: %lf, y: %lf", index, odom_path.poses[index].pose.position.x, odom_path.poses[index].pose.position.x);
            }
        }
    }

    double getCurvature(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3)
    {
        double curvature = 0;
        if((p1.x == p2.x && p2.x == p3.x) || (fabs((p2.y - p1.y) / (p2.x - p1.x) - (p3.y - p2.y) / (p3.x - p2.x))) < 0.000001)
        {
            curvature = 0; 
        }
        else
        {
            double radius;//曲率半径
            double dis, dis1, dis2, dis3;//距离
            double cosA;//ab确定的边所对应的角A的cos值
            dis1 = getDistanceBetweenPoints(p1, p2);
            dis2 = getDistanceBetweenPoints(p1, p3);
            dis3 = getDistanceBetweenPoints(p2, p3);
            dis = dis2 * dis2 + dis3 * dis3 - dis1 * dis1;
            cosA = dis / ( 2 * dis2 * dis3);//余弦定理
            radius = 0.5 * dis1 / cosA;
            curvature = 1 / radius;
        }
        return curvature;
    }

    bool findPredictedCarPose(const geometry_msgs::Pose& carPose, geometry_msgs::Pose & carPosePredicted, geometry_msgs::Pose & distance)
    {
        foundPredictedCarPt = false;
        geometry_msgs::Point carPose_pos = carPose.position;
        double carPose_yaw = getYawFromPose(carPose);
        bool foundCarInWay = false;
        geometry_msgs::Pose carPoseInWay;

        if(!goal_reached)
        {
            for(int i = 0; i< odom_path.poses.size(); i++) // 对 path 上的每一个点进行处理
            {
                geometry_msgs::PoseStamped odom_path_pose = odom_path.poses[i];
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;

                if(isForwardWayPt(odom_path_wayPt, carPose))
                {
                    if(!foundCarInWay)
                    {
                        foundCarInWay = true;
                        carPoseInWay = odom_path.poses[i].pose;
                        // ROS_ERROR("carPoseInWay: x = %lf, y = %lf, yaw = %lf", carPoseInWay.position.x, carPoseInWay.position.y, getYawFromPose(carPoseInWay));
                    }
                    double dist = getDistanceBetweenPoints(odom_path_wayPt, carPose_pos);
                    if(dist > predicted_dist)
                    {
                        foundPredictedCarPt = true;
                        carPosePredicted = odom_path.poses[i].pose;
                        // ROS_ERROR("carPosePredicted: x = %lf, y = %lf, yaw = %lf", carPosePredicted.position.x, carPosePredicted.position.y, getYawFromPose(carPosePredicted));
                        distance.position.x = carPosePredicted.position.x - carPoseInWay.position.x;
                        distance.position.y = carPosePredicted.position.y - carPoseInWay.position.y;
                        double yaw = getYawFromPose(carPosePredicted) - getYawFromPose(carPoseInWay);
                        tf::Quaternion quaternion = tf::createQuaternionFromYaw(yaw);
                        distance.orientation.w = quaternion.w();
                        distance.orientation.x = quaternion.x();
                        distance.orientation.y = quaternion.y();
                        distance.orientation.z = quaternion.z();
                        break;
                    }
                }
            }
        }
        return foundPredictedCarPt;
    }

    // 找到规划出来的路径中转角与机器人的距离
    double findDistanceToBend(const geometry_msgs::Pose& carPose)
    {
        double distance = 0;
        points.points.clear();
        double cost = 0, x_diff, y_diff, yaw, yaw_last;

        if(!goal_reached)
        {
            for(int index = 2; index < odom_path.poses.size(); index++)
            {
                geometry_msgs::PoseStamped odom_path_pose = odom_path.poses[index];
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;

                if(isForwardWayPt(odom_path_wayPt, carPose))
                {
                    yaw_last = atan2(odom_path.poses[index -1].pose.position.y - odom_path.poses[index - 2].pose.position.y, odom_path.poses[index - 1].pose.position.x - odom_path.poses[index - 2].pose.position.x);
                    x_diff = odom_path.poses[index].pose.position.x - odom_path.poses[index - 1].pose.position.x;
                    y_diff = odom_path.poses[index].pose.position.y - odom_path.poses[index - 1].pose.position.y;
                    yaw = atan2(y_diff, x_diff);

                    if(fabs(yaw + 2 * PI - yaw_last) < fabs(yaw - yaw_last)) cost += (fabs(yaw + 2 * PI - yaw_last));
                    else if(fabs(yaw - 2 * PI - yaw_last) < fabs(yaw - yaw_last)) cost += (fabs(yaw -  2 * PI - yaw_last));
                    else cost += fabs(yaw - yaw_last);
                    distance = getDistanceBetweenPoints(odom_path_wayPt, carPose.position);

                    if(cost < cost_max) points.points.push_back(odom_path_wayPt);
                    else break;
                }
            }
        }
        marker_pub.publish(points);

        return distance;
    }

    double getExpectedSpeed(double distance)
    {
        double result;
        ros::Time currentTime = ros::Time::now();
        ros::Duration interval = currentTime - stopTime;

        if(distance < slowdown_dist_stop && interval < ros::Duration(stop_interval, 0))
        {
            result = Vcmd_stop;
            stopTime = currentTime;
        }
        else
        {
            if(distance >= slowdown_dist_max) result = Vcmd_max;
            else if(distance <= slowdown_dist_min) result = Vcmd_min;
            else result = Vcmd_min + (Vcmd_max - Vcmd_min) / (slowdown_dist_max - slowdown_dist_min) * (distance - slowdown_dist_min);                                                                                                                                                                                                                                                                                                                  
        }
        return result * speed_factor;
    }

    geometry_msgs::Point findForwardPoint(const geometry_msgs::Pose& carPose)
    {
        geometry_msgs::Point carPose_pos = carPose.position;
        double carPose_yaw = getYawFromPose(carPose);
        geometry_msgs::Point forwardPt;
        geometry_msgs::Point odom_car2WayPtVec;
        foundForwardPt = false;
        int index = 0;

        double cost = 0, x_diff, y_diff, yaw, yaw_last;

        if(!goal_reached)
        {

            for(int i = 0; i< odom_path.poses.size(); i++) // 对 path 上的每一个点进行处理
            {
                geometry_msgs::PoseStamped odom_path_pose = odom_path.poses[i];
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;

                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt, carPose);
                
                if(_isForwardWayPt)
                {
                    double dist = getDistanceBetweenPoints(odom_path_wayPt, carPose_pos);

                    if(!foundForwardPt) // 如果 odom_path_wayPt 在 carPosePredicted 的前面
                    {
                        if(dist > Lfw)
                        {
                            forwardPt = odom_path_wayPt; // 如果 odom_path_wayPt 满足在 carPosePredicted 的前面,并且与 carPosePredicted 的距离大于参数 Lfw,记录满足条件的的一个点
                            foundForwardPt = true;
                            PURE_PURSUIT_INFO("found the forwardPt, index: %d, x: %lf, y: %lf", index, forwardPt.x, forwardPt.y);
                            break;
                        }
                    }
                    index++;
                }
            }
        }
        else if(goal_reached)
        {
            forwardPt = odom_goal_pos; // 如果已经接近终点,那么设定 forwardPt 为 odom_goal_pos
            foundForwardPt = false;
            //ROS_INFO("goal REACHED!");
        }
        
        return forwardPt; // 返回 forwardPt 与 carPose_pos 的距离并转换到机器人的坐标系下
    }

    double getSteering(double eta)
    {
        return atan2((this->L*sin(eta)),(this->Lfw/2 + this->lfw*cos(eta)));
    }

    // 接受到机器人在 map 坐标系下的坐标,判断机器人是否已经到达目标点
    void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
    {
        if(this->goal_received)
        {
            std_srvs::SetBool trigger;
            trigger.request.data = true;
            double car2goal_x = this->goal_pos.x - amclMsg->pose.pose.position.x;
            double car2goal_y = this->goal_pos.y - amclMsg->pose.pose.position.y;
            double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
            if(dist2goal < this->goal_radius)
            {
                this->goal_reached = true;
                this->goal_received = false;
                ROS_INFO("Goal Reached !");
                stopRobotCB(trigger.request, trigger.response);
            }
            if(enbale_safe_distance && isRobotCollided(amclMsg->pose.pose.position))
            {
                ROS_ERROR("robot hit the obstacle");
                stopRobotCB(trigger.request, trigger.response);
            }
        }
    }

    void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& data)
    {
        costmap = data;
    }

    void speedFactorCB(const std_msgs::Float32::ConstPtr& data)
    {
        speed_factor = data->data;
    }

    bool isRobotCollided(geometry_msgs::Point carPointMap)
    {
        bool result = false;
        //ROS_ERROR("costmap->data.size(): %ld", costmap->data.size());
        double resolution = costmap->info.resolution;
        int dx = (int)((carPointMap.x - costmap->info.origin.position.x) / resolution + 0.5);
        int dxMax = std::min(dx + (int)(safe_distance_x / resolution + 0.5), (int)costmap->info.width - 1);
        int dxMin = std::max(dx - (int)(safe_distance_x / resolution + 0.5), (int)0);
        //ROS_ERROR("dx: %d, dxMax: %d, dxMin: %d", dx, dxMax, dxMin);

        int dy = (int)((carPointMap.y - costmap->info.origin.position.y) / resolution + 0.5);
        int dyMax = std::min(dy + (int)(safe_distance_y / resolution + 0.5), (int)costmap->info.height - 1);
        int dyMin = std::max(dy - (int)(safe_distance_y / resolution + 0.5), (int)0);
        //ROS_ERROR("dy: %d, dyMax: %d, dyMin: %d", dy, dyMax, dyMin);

        for(long y = dyMin; y <= dyMax; ++y)
        {
            for(long x = dxMin; x <= dxMax; ++x)
            {
                long index = y * costmap->info.width + x;
                if(costmap->data[index] > 0)
                {
                    result = true;
                    break;
                }
            }
        }
        
        return result;
    }

    // 控制周期
    void controlLoopCB(const ros::TimerEvent&)
    {
        if(!goal_received) return;

        geometry_msgs::Pose carPose = this->odom.pose.pose;
        geometry_msgs::Twist carVel = this->odom.twist.twist;
        double carPoseYaw = getYawFromPose(carPose);

        if(this->goal_received)
        {
            PURE_PURSUIT_INFO("============= controlLoopCB =============");
            PURE_PURSUIT_INFO("current car pose: x: %lf, y: %lf, vx: %lf, vyaw: %lf", carPose.position.x, carPose.position.y, carVel.linear.x, carVel.angular.z);
            transformPathData();

            geometry_msgs::Pose carPosePredicted, predictedDistance;
            if(findPredictedCarPose(carPose, carPosePredicted, predictedDistance))
            {
                PURE_PURSUIT_INFO("found the predicted car pose, x: %lf, y: %lf", carPosePredicted.position.x, carPosePredicted.position.y);
                carPosePredicted.orientation = carPose.orientation;
                // carPoseCorrected = carPose;
                // carPoseCorrected.position.x += (cos(carPoseYaw) * predictedDistance.position.x + sin(carPoseYaw) * predictedDistance.position.y);
                // carPoseCorrected.position.y += (-sin(carPoseYaw) * predictedDistance.position.x + cos(carPoseYaw) * predictedDistance.position.y);
            }
            else 
            {
                PURE_PURSUIT_INFO("didn't find the predicted car pose!!!!!!!!!!!");
                carPosePredicted = carPose;
                // carPoseCorrected = carPose;
            }


            double bendDistance = findDistanceToBend(carPose);
            speed_expected = getExpectedSpeed(bendDistance);

            /*Estimate Steering Angle*/
            geometry_msgs::Point odom_car2WayPtVec, forwardPt = findForwardPoint(carPosePredicted);

            /*Visualized Target Point on RVIZ*/
            /*Clear former target point Marker*/
            lines.points.clear();
            
            if(foundForwardPt && !goal_reached) // 使用 Marker 将 forwardPt 和 carPose_pos 标记出来
            {
                lines.points.push_back(carPosePredicted.position);
                lines.points.push_back(forwardPt);
            }
            else ROS_ERROR("foundForwardPt%d", (int)foundForwardPt);
            marker_pub.publish(lines);

            odom_car2WayPtVec.x = cos(carPoseYaw)*(forwardPt.x - carPosePredicted.position.x) + sin(carPoseYaw) * (forwardPt.y - carPosePredicted.position.y);
            odom_car2WayPtVec.y = -sin(carPoseYaw)*(forwardPt.x - carPosePredicted.position.x) + cos(carPoseYaw) * ( forwardPt.y - carPosePredicted.position.y);
            double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x); // 计算 forwardPt 与 carPose_pos 的角度
            
            if(foundPredictedCarPt && foundForwardPt)
            {
                this->steering = this->base_angle + getSteering(eta) * this->steering_gain; // 计算舵机转角
                this->steering = std::min(this->steering,  steering_max);
                this->steering = std::max(this->steering, -steering_max);

                /*Estimate Gas Input*/
                if(!this->goal_reached)
                {
                    if(this->smooth_accel) // 计算速度
                    {
                        if(speed_expected >= this->velocity) this->velocity = std::min(this->velocity + this->speed_incremental, speed_expected);
                        else this->velocity = speed_expected;
                    }
                    else
                    {
                        this->velocity = speed_expected;
                    }
                    if(debug_mode) ROS_INFO("Velocity = %.2f, Steering = %.2f", this->velocity, this->steering);
                }
            }
        }

        if(this->goal_reached)
        {
            this->velocity = 0.0;
            this->steering = this->base_angle;
        }
        
        if(!stop_robot)
        {
            this->ackermann_cmd.header.stamp = ros::Time::now();
            this->ackermann_cmd.drive.steering_angle = this->steering;
            this->ackermann_cmd.drive.speed = this->velocity;
            this->ackermann_pub.publish(this->ackermann_cmd);

            if(this->cmd_vel_mode)
            {
                this->cmd_vel.linear.x = this->velocity;
                this->cmd_vel.angular.z = this->steering;
                this->cmdvel_pub.publish(this->cmd_vel);
            }
        }
    }

    bool stopRobotCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if(!stop_robot && req.data)
        {
            this->ackermann_cmd.header.stamp = ros::Time::now();
            this->ackermann_cmd.drive.steering_angle = this->steering;
            this->ackermann_cmd.drive.speed = 0;
            this->ackermann_pub.publish(this->ackermann_cmd);
        }

        stop_robot = req.data;
        res.success = stop_robot;
        res.message = "success"; 
        return true;
    }
}; // end of class


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "PurePursuit");
    PurePursuit controller;
    ros::AsyncSpinner spinner(2); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
