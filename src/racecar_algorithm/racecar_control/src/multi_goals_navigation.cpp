#include <iostream>
#include <string>
#include <queue>
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
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

#define PI 3.1415926535898

// 用于实现 Dijkstra 算法的类
struct Dijkstra
{
   
    // 用于表示两个坐标点直接的关系的类
    struct LocationsRelation
    {
        int from; // from 坐标的序号
        int to; // to 坐标的序号
        const geometry_msgs::Point * fromPoint; // from 坐标的数据
        const geometry_msgs::Point * toPoint; // to 坐标的数据
        double punish = 1.0;

        LocationsRelation (int fromIndex = 0, int toIndex = 0, const geometry_msgs::Point * pFromPoint = nullptr, const geometry_msgs::Point * pToPoint = nullptr, double punishRate = 1.0) :
        from(fromIndex), to(toIndex), fromPoint(pFromPoint), toPoint(pToPoint), punish(punishRate) {}
        double dist(void) const
        {
            double dx = fromPoint->x - toPoint->x;
            double dy = fromPoint->y - toPoint->y;
            return sqrt(dx * dx + dy * dy);
        }
        double angle(void) const { return atan2(toPoint->y - fromPoint->y, toPoint->x - fromPoint->x); }
    };
    

    // Dijkstra算法用到的优先队列的节点
    struct HeapNode
    {
        double cost; // 从起点到当前点的 cost
        double dist;
        double angle;
        int index; // 当前点的序号
        const LocationsRelation* relation; // 当前的的 LocationsRelation
        std::vector<int> path; // 记录从起点到当前点的路径
        HeapNode(int i, double c, double d, double a, const LocationsRelation* r = nullptr, std::vector<int> p = std::vector<int>()) :
        index(i), cost(c), dist(d), angle(a), relation(r), path(p) {}
        bool operator < (const HeapNode &rhs) const
        {
            return cost > rhs.cost;
        }
    };

    // 初始化
    void init(int maxLocations,const std::vector<LocationsRelation>& lr)
    {
        maxLocationsNum = maxLocations;
        reachableIndexs.clear();

        for(int index = 0; index < maxLocationsNum; ++index) reachableIndexs.push_back(std::vector<int>());
        
        locationsRelation.clear();
        for(int index = 0; index < lr.size(); ++index)
        {
            locationsRelation.push_back(lr[index]);
            reachableIndexs[lr[index].from].push_back(locationsRelation.size() - 1);

            locationsRelation.push_back(LocationsRelation(lr[index].to, lr[index].from, lr[index].toPoint, lr[index].fromPoint));
            reachableIndexs[lr[index].to].push_back(locationsRelation.size() - 1);
        }
    }

    // 运行 Dijkstra 算法
    void run(int startIndex, double startAngle, double * distance, double * angle, double * cost, std::vector<int> * bestPath) const
    {
        std::priority_queue<HeapNode> heapNodeQueue;
        const int maxCost = 10000;
        bool reached[maxLocationsNum];

        for(int index = 0; index < maxLocationsNum; ++index)
        {
            reached[index] = false;
            cost[index] = maxCost;
        }
        distance[startIndex] = 0;
        angle[startIndex] = 0;
        cost[startIndex] = 0;
        HeapNode startNode(startIndex, 0, 0, 0);
        startNode.path.clear();
        startNode.path.push_back(startIndex);
        bestPath[startIndex] = startNode.path;
        heapNodeQueue.push(startNode);

        while(!heapNodeQueue.empty())
        {
            HeapNode currentNode = heapNodeQueue.top();
            heapNodeQueue.pop();

            int currentIndex = currentNode.index;
            const LocationsRelation* lastRelation = currentNode.relation;
            std::vector<int> currentPath = currentNode.path;

            if(reached[currentIndex]) continue;
            reached[currentIndex] = true;

            for(int temp = 0; temp < reachableIndexs[currentIndex].size(); ++temp)
            {
                const LocationsRelation* relation = &locationsRelation[reachableIndexs[currentIndex][temp]];
                double relationAngle = 0;
                
                if(lastRelation != nullptr)
                {
                    if(lastRelation->from == relation->to) relationAngle = punishBackwards; // 如果是往返,直接等于2PI 
                    else // 否则不会大于PI
                    {
                        relationAngle = fabs(relation->angle() - lastRelation->angle());
                        if(relationAngle >  (PI + 0.000001)) relationAngle -= PI;
                        if(relationAngle < -(PI + 0.000001)) relationAngle += PI;
                    }
                }
                else
                {
                    relationAngle = fabs(relation->angle() - startAngle);
                    if(relationAngle >  (PI + 0.000001)) relationAngle -= PI;
                    if(relationAngle < -(PI + 0.000001)) relationAngle += PI;
                    if(fabs(relationAngle) > PI * 3.0 / 4.0) relationAngle *= punishBackwards; // 如果第一个点的转弯角度大于135,那么认为是倒退
                }
                
                double relationCost = (relation->dist() + relationAngle * punishBent) * relation->punish;
                if(cost[relation->to] > cost[relation->from] + relationCost)
                {
                    cost[relation->to] = cost[relation->from] + relationCost;
                    distance[relation->to] = distance[relation->from] + relation->dist();
                    angle[relation->to] = angle[relation->from] + relationAngle;
                    HeapNode nextNode(relation->to, cost[relation->to], distance[relation->to], angle[relation->to], relation, currentPath);
                    nextNode.path.push_back(relation->to);
                    bestPath[relation->to] = nextNode.path;
                    heapNodeQueue.push(nextNode);
                }
            }
        }
    }

    std::vector<LocationsRelation> locationsRelation; // 代表坐标点的关系
    std::vector<std::vector<int> > reachableIndexs; // 代表每个坐标点可到达的其他坐标点的序号
    double punishBent = 0.0;
    double punishBackwards = 0.0;
    int maxLocationsNum; // 坐标点的数量
};

class MultiGoalsNavigation
{
private:

    ros::NodeHandle nodeHandle;
    ros::Subscriber amclSubscriber, goalSubscriber, clickedPointSubscriber, globalCostmapSubscriber;
    ros::Publisher goalPublisher, markerPublisher;
    ros::ServiceServer normalizeServer;
    ros::ServiceClient clearCostmapsClient;

    std::string mapFrame, clearCostmapsServer ,amclPoseTopic, initPoseTopic, goalTopic, markerTopic, clickedPointTopic, golbalCostmapTopic;
    double goalRadius, mapOffsetX, mapOffsetY, mapOffsetYaw, punishBend, punishBackwards;
    bool debugMode, navigationStarted = false;
    visualization_msgs::Marker goalsMarker, roadMarker;

    XmlRpc::XmlRpcValue locationsXmlRpc, locationsRelationXmlRpc, goalsIndexXmlRpc, goalsStaticXmlRpc;

    std::vector<geometry_msgs::Point> locationVector; // 记录各个坐标点的坐标数据,从param中读取
    std::vector<Dijkstra::LocationsRelation> locationRelationVector; // 记录两两坐标点之间的关系,从param中读取
    std::vector<int> goalsIndexVector; // 记录目标在 locationVector 中的序号,从param中读取
    std::vector<bool> goalsStaticVector; // 根据该设置对目标点进行排序,从param中读取
    std::vector<int> optimalGoalsIndexVector; // 优化后得到的目标店序列

    int currentGoalIndex = 0; // 当前目标的序号
    const geometry_msgs::Point* currentGoal; // 当前坐标的数据
    geometry_msgs::PoseStamped publishedGoal; // currentGoal 经过归一化操作(平移、旋转)后,真正发布的坐标信息

    #define NORMALIZED_POINTS_NUMBER 3
    geometry_msgs::Point clickedPoints[NORMALIZED_POINTS_NUMBER]; // 进行归一化操作时,用于保存标定点数据
    int normalizedIndex = 0; // 记录表定点个数,当该值等于 NORMALIZED_POINTS_NUMBER 时,开始归一化
    bool normalizeStarted = false;

    Dijkstra dijkstra;

public:
    ~MultiGoalsNavigation() = default;
    MultiGoalsNavigation(ros::NodeHandle node)
    {
        nodeHandle = node;
        ros::NodeHandle _n("~");

        _n.param<std::string>("map_frame", mapFrame, "map");
        _n.param<std::string>("clear_costmaps_server", clearCostmapsServer, "/move_base/clear_costmaps");
        _n.param<std::string>("amcl_pose_topic", amclPoseTopic, "/amcl_pose");
        _n.param<std::string>("init_pose_topic", initPoseTopic, "/initialpose");
        _n.param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");
        _n.param<std::string>("marker_topic", markerTopic, "visualization_marker");
        _n.param<std::string>("clicked_point_topic", clickedPointTopic, "/clicked_point");
        _n.param<std::string>("global_costmap_topic", golbalCostmapTopic, "/move_base/global_costmap/costmap");

        _n.param<double>("goal_radius", goalRadius, 1.0);
        _n.param<double>("map_offset_x", mapOffsetX, 0.0);
        _n.param<double>("map_offset_y", mapOffsetY, 0.0);
        _n.param<double>("map_offset_yaw", mapOffsetYaw, 0.0);
        _n.param<double>("punish_bend", punishBend, 0.0);
        _n.param<double>("punish_backwards", punishBackwards, 0.0);
        dijkstra.punishBent = punishBend;
        dijkstra.punishBackwards = punishBackwards;

        _n.param<bool>("debug_mode", debugMode, false);

        _n.getParam("locations", locationsXmlRpc);
        _n.getParam("locations_relation", locationsRelationXmlRpc);
        _n.getParam("goals_id", goalsIndexXmlRpc);
        _n.getParam("goals_static", goalsStaticXmlRpc);

        initMarker();

        amclSubscriber = nodeHandle.subscribe(amclPoseTopic, 1, &MultiGoalsNavigation::amclPoseCallBack, this);
        goalSubscriber = nodeHandle.subscribe(initPoseTopic, 1, &MultiGoalsNavigation::initPoseoalCallBack, this);
        //globalCostmapSubscriber = nodeHandle.subscribe(golbalCostmapTopic, 1, &MultiGoalsNavigation::costmapCallBack, this);
        clickedPointSubscriber.shutdown();
        goalPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>(goalTopic, 10);
        markerPublisher = nodeHandle.advertise<visualization_msgs::Marker>(markerTopic, 10);
        normalizeServer = nodeHandle.advertiseService("start_normalize_map", &MultiGoalsNavigation::startNormalizeMapCallBack, this);
        clearCostmapsClient = nodeHandle.serviceClient<std_srvs::Empty>(clearCostmapsServer);

        try
        {
            locationVector = getPointVectorFromXMLRPC(locationsXmlRpc, "locations");
            locationRelationVector = getRelationVectorFromXMLRPC(locationsRelationXmlRpc, "locations_relation");
            goalsIndexVector = getNumberVectorFromXMLRPC<int>(goalsIndexXmlRpc, "goals_id");
            goalsStaticVector = getNumberVectorFromXMLRPC<bool>(goalsStaticXmlRpc, "goals_static");
        } 
        catch(const std::exception& ex)
        {
            checkCondition(true, ex.what());
        }

        checkCondition(goalsIndexVector.size() != goalsStaticVector.size(), "fatal error: the size of goals_id array param is not equal to the size of goals_static array param");
        checkCondition(goalsIndexVector.size() == 0, "fatal error: the size of goals_id array param is 0");
        checkCondition(!(goalsStaticVector[0] && goalsStaticVector[goalsStaticVector.size() - 1]), "fatal error: the begin and the end of goals_static array param must be true");

        if(debugMode) ROS_INFO("locations relation:");
        for(auto index = 0; index < locationRelationVector.size(); ++index)
        {
            int formIndex = --locationRelationVector[index].from;
            int toIndex = --locationRelationVector[index].to;

            checkCondition(formIndex > locationVector.size() - 1, "fatal error: locations_relation param is wrong");
            locationRelationVector[index].fromPoint = &locationVector[formIndex];
            checkCondition(toIndex > locationVector.size() - 1, "fatal error: locations_relation param is wrong");
            locationRelationVector[index].toPoint = &locationVector[toIndex];
            if(debugMode) ROS_INFO("form: %d, to: %d, punish: %lf", formIndex + 1, toIndex + 1, locationRelationVector[index].punish);
        }

        if(debugMode) ROS_INFO("goals info:");
        for(auto index = 0; index < goalsIndexVector.size(); ++index)
        {
            goalsIndexVector[index] -= 1;
            checkCondition(goalsIndexVector[index] > locationVector.size() - 1, "fatal error: some elements in goals_id array param is large than the size of locations array param");
            if(debugMode) ROS_INFO("index: %d, id: %d, static: %d, goal.x: %lf, goal.y: %lf", index, goalsIndexVector[index] + 1, (int)goalsStaticVector[index], 
                locationVector[goalsIndexVector[index]].x, locationVector[goalsIndexVector[index]].y);
        }

        dijkstra.init(locationVector.size(), locationRelationVector);
 
        if(debugMode)
        {
            ROS_INFO("dijkstra results:");
            for(int startIndex = 0; startIndex < locationVector.size(); ++startIndex)
            {
                double testDistance[locationVector.size()];
                double testAngle[locationVector.size()];
                double testCost[locationVector.size()];
                std::vector<int> bestPath[locationVector.size()];
                for(int temp = 0; temp < dijkstra.reachableIndexs[startIndex].size(); ++temp)
                {
                    const Dijkstra::LocationsRelation * relation = &dijkstra.locationsRelation[dijkstra.reachableIndexs[startIndex][temp]];
                    double yaw = atan2(relation->fromPoint->y - relation->toPoint->y, relation->fromPoint->x - relation->toPoint->x);
                    dijkstra.run(startIndex, yaw, testDistance, testAngle, testCost, bestPath);
                    ROS_INFO("#################################");
                    ROS_INFO("start point index: %d, start oriantation: %d -> %d, yaw: %lf", startIndex + 1, relation->to + 1, relation->from + 1, yaw);
                    for(int endIndex = 0; endIndex < locationVector.size(); ++endIndex)
                    {
                        std::string str = std::string("end point index: ") + std::to_string(endIndex + 1) + 
                                        std::string(" distance: ") + std::to_string(testDistance[endIndex]) + 
                                        std::string(" angle: ") + std::to_string(testAngle[endIndex]) + 
                                        std::string(" cost: ") + std::to_string(testCost[endIndex]);
                        str += std::string(" best path: ");
                        for(int index = 0; index < bestPath[endIndex].size(); ++index)
                        {
                            str = str + std::to_string(bestPath[endIndex][index] + 1) + std::string(" -> ");
                        }
                        str += std::string("end");
                        ROS_INFO_STREAM(str);
                    }
                }
            }
        }

        ROS_INFO("print optimal goals with all possible oriantations");
        for(int index = 0; index < dijkstra.reachableIndexs[goalsIndexVector[0]].size(); ++index)
        {
            const Dijkstra::LocationsRelation * relation = &dijkstra.locationsRelation[dijkstra.reachableIndexs[goalsIndexVector[0]][index]];
            double yaw = atan2(relation->fromPoint->y - relation->toPoint->y, relation->fromPoint->x - relation->toPoint->x);

            std::vector<int> testOptimalGoals = getOptimalGoalsIndex(yaw, goalsIndexVector);

            ROS_INFO("start point index: %d, start oriantation: %d -> %d, yaw: %lf", goalsIndexVector[0] + 1, relation->to + 1, relation->from + 1, yaw);
            std::string str;
            str += std::string(" best path: ");
            for(int index = 0; index < testOptimalGoals.size(); ++index)
            {
                str = str + std::to_string(testOptimalGoals[index] + 1) + std::string(" -> ");
            }
            str += std::string("end");
            ROS_INFO_STREAM(str);
        }
    }

    void initMarker(void)
    {
        goalsMarker.header.frame_id = roadMarker.header.frame_id = mapFrame;
        goalsMarker.ns = roadMarker.ns = "multi_goals_navigation";
        goalsMarker.action = roadMarker.action = visualization_msgs::Marker::ADD;
        tf::Quaternion quaternion = tf::createQuaternionFromYaw(-mapOffsetYaw);
        goalsMarker.pose.orientation.w = roadMarker.pose.orientation.w = 1;

        goalsMarker.id = 0;
        goalsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        goalsMarker.scale.x = goalRadius;
        goalsMarker.scale.y = goalRadius;
        goalsMarker.scale.y = 0.01;
        goalsMarker.color.r = 1.0;
        goalsMarker.color.g = 1.0;
        goalsMarker.color.b = 0.0;
        goalsMarker.color.a = 0.3;

        roadMarker.id = 1;
        roadMarker.type = visualization_msgs::Marker::LINE_STRIP;
        roadMarker.scale.x = 0.05;
        roadMarker.color.g = 1.0;
        roadMarker.color.a = 0.3;
    }

    void publishGoalMarker(const std::vector<int> & goalsIndex)
    {
        goalsMarker.points.clear();
        for(auto index = 0; index < goalsIndex.size(); ++index)
        {
            geometry_msgs::Point goal = normalizePoint(locationVector[goalsIndex[index]].x, locationVector[goalsIndex[index]].y);
            goalsMarker.points.push_back(goal);
        }
        markerPublisher.publish(goalsMarker);
    }

    void publishRoadMarker(const std::vector<int> & optimalGoalsIndex)
    {
        roadMarker.points.clear();
        for(auto index = 0; index < optimalGoalsIndex.size(); ++index)
        {
            geometry_msgs::Point road = normalizePoint(locationVector[optimalGoalsIndex[index]].x, locationVector[optimalGoalsIndex[index]].y);
            roadMarker.points.push_back(road);
        }
        markerPublisher.publish(roadMarker);
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

    geometry_msgs::Point normalizePoint(const double xorigin, const double yOrigin) const
    {
        geometry_msgs::Point result;

        double x = xorigin - mapOffsetX;
        double y = yOrigin - mapOffsetY;
        double radius = sqrt(x * x + y * y);
        double yaw = atan2(y, x);
        result.x = radius * cos(yaw) * cos(-mapOffsetYaw) - radius * sin(yaw) * sin(-mapOffsetYaw);
        result.y = radius * sin(yaw) * cos(-mapOffsetYaw) + radius * cos(yaw) * sin(-mapOffsetYaw);
        
        return result;
    }

    void publishGoal(int currentGoalIndex)
    {
        currentGoal = &locationVector[optimalGoalsIndexVector[currentGoalIndex]];
        if(debugMode) ROS_INFO("current goal info (before rotation) index: %d, id: %d, x: %lf, y: %lf", currentGoalIndex, optimalGoalsIndexVector[currentGoalIndex] + 1, currentGoal->x, currentGoal->y);

        const geometry_msgs::Point * lastGoal = &locationVector[optimalGoalsIndexVector[currentGoalIndex - 1]];
        double currentAngel = atan2(currentGoal->y - lastGoal->y, currentGoal->x - lastGoal->x);

        double xPublished = currentGoal->x, yPublished = currentGoal->y, anglePublished = currentAngel;
        if(currentGoalIndex + 1 < optimalGoalsIndexVector.size() - 1)
        {
            const geometry_msgs::Point * futureGoal = &locationVector[optimalGoalsIndexVector[currentGoalIndex + 1]];
            anglePublished = atan2(futureGoal->y - currentGoal->y, futureGoal->x - currentGoal->x);
            xPublished += (cos(anglePublished) * goalRadius);
            yPublished += (sin(anglePublished) * goalRadius);
        }
        
        geometry_msgs::Point normalizedResult = normalizePoint(xPublished, yPublished);
        double rotatedYaw = anglePublished - mapOffsetYaw;
        if(rotatedYaw > PI) rotatedYaw -= (2 * PI);
        if(rotatedYaw < -PI) rotatedYaw += (2 * PI);

        publishedGoal = geometry_msgs::PoseStamped();
        tf::Quaternion q = tf::createQuaternionFromYaw(rotatedYaw);

        publishedGoal.header.frame_id = mapFrame;
        publishedGoal.header.stamp = ros::Time::now();
        publishedGoal.pose.position.x = normalizedResult.x;
        publishedGoal.pose.position.y = normalizedResult.y;
        publishedGoal.pose.position.z = 0;
        publishedGoal.pose.orientation.w = q.w();
        publishedGoal.pose.orientation.x = q.x();
        publishedGoal.pose.orientation.y = q.y();
        publishedGoal.pose.orientation.z = q.z();
        if(debugMode) ROS_INFO("current goal info (after rotation) index: %d, id: %d, x: %lf, y: %lf", currentGoalIndex, optimalGoalsIndexVector[currentGoalIndex] + 1, normalizedResult.x, normalizedResult.y);

        goalPublisher.publish(publishedGoal);
    }

    void numberPermutation(std::vector<int> numberVector, int begin, int end, std::vector<std::vector<int> > * resultVector) const
    {
        if(begin == end) resultVector->push_back(numberVector);

        for(int index = begin; index < end; ++index)
        {
            int temp = numberVector[begin];
            numberVector[begin] = numberVector[index];
            numberVector[index] = temp;

            numberPermutation(numberVector, begin + 1, end, resultVector);

            temp = numberVector[begin];
            numberVector[begin] = numberVector[index];
            numberVector[index] = temp;
        }
    }

    std::vector<int> choooceBestPath(int startIndex, double startYaw, int stopIndex, const std::vector<int>& goalsIndex) const
    {
        std::vector<std::vector<int> > goalsPermutation;
        std::vector<int> tempGoalsIndex = goalsIndex;
        numberPermutation(tempGoalsIndex, 0, tempGoalsIndex.size(), &goalsPermutation);

        double minCost = 1000000;
        int minIndex = 0;
        std::vector<int> result;
        for(int index = 0; index < goalsPermutation.size(); ++index) // 对所有可能性进行遍历
        {
            std::vector<int> goalsSelected = goalsPermutation[index]; // 得到排列组合后的序列,并加上起点终点信息
            goalsSelected.insert(goalsSelected.begin(), startIndex);
            goalsSelected.push_back(stopIndex);

            double costSum = 0;
            std::vector<int> pathSum;
            for(int alpha = 1; alpha < goalsSelected.size(); ++alpha)
            {
                const int begin = goalsSelected[alpha - 1];
                const int end = goalsSelected[alpha];
                double oriantation = startYaw;
                if(alpha >= 2)
                {
                    const geometry_msgs::Point * fromPoint = &locationVector[goalsSelected[alpha - 2]];
                    const geometry_msgs::Point * toPoint = &locationVector[goalsSelected[alpha - 1]];
                    oriantation = atan2(toPoint->y - fromPoint->y, toPoint->x - fromPoint->x);
                }

                double bestDistance[locationVector.size()];
                double bestAngle[locationVector.size()];
                double bestCost[locationVector.size()];
                std::vector<int> bestPath[locationVector.size()];
                dijkstra.run(begin, oriantation, bestDistance, bestAngle, bestCost, bestPath);

                costSum += bestCost[end];
                for(int beta = 1; beta < bestPath[end].size(); ++beta)
                {
                    pathSum.push_back(bestPath[end][beta]);
                }
            }

            if(costSum < minCost)
            {
                minCost = costSum;
                minIndex = index;
                result = pathSum;
            }
        }

        result.insert(result.begin(), startIndex);
        return result;
    }

    std::vector<int> getOptimalGoalsIndex(const double startYaw, const std::vector<int>& goalsIndex) const
    {
        std::vector<int> goalsNeedOptimize;
        std::vector<int> goalsOptimized;

        goalsOptimized.push_back(goalsIndex[0]); // 第一个点一定是固定的起点
        for(int index = 1; index < goalsIndexVector.size(); ++index)
        {
            if(!goalsStaticVector[index])
            {
                goalsNeedOptimize.push_back(goalsIndexVector[index]);
            }
            else
            {
                int startIndex = goalsOptimized[goalsOptimized.size() - 1]; // 优化队列中最后一个值当做起点
                int endIndex = goalsIndexVector[index]; // 当前值当做终点
                double oriantation = startYaw;
                if(goalsOptimized.size() >= 2)
                {
                    const geometry_msgs::Point * fromPoint = &locationVector[goalsOptimized[goalsOptimized.size() - 2]];
                    const geometry_msgs::Point * toPoint = &locationVector[goalsOptimized[goalsOptimized.size() - 1]];
                    oriantation = atan2(toPoint->y - fromPoint->y, toPoint->x - fromPoint->x);
                }
                std::vector<int> result = choooceBestPath(startIndex, oriantation, endIndex, goalsNeedOptimize);
                for(int temp = 1; temp < result.size(); ++temp) goalsOptimized.push_back(result[temp]);
                goalsNeedOptimize.clear();
            }
        }

        return goalsOptimized;
    }

    void clickedPointCallBack(const geometry_msgs::PointStamped::ConstPtr& point)
    {
        clickedPoints[normalizedIndex] = point->point;
        ROS_INFO("normalized info: points, index: %d, x: %lf, y: %lf", normalizedIndex, point->point.x, point->point.y);
        ++normalizedIndex;

        if(normalizedIndex == NORMALIZED_POINTS_NUMBER)
        {
            clickedPointSubscriber.shutdown();
            geometry_msgs::Point idealPoints[NORMALIZED_POINTS_NUMBER];
            idealPoints[0].x = -8, idealPoints[0].y = 5;
            idealPoints[1].x = 9,  idealPoints[1].y = 5;
            idealPoints[2].x = 9,  idealPoints[2].y = -5.5;

            double idealYaw[NORMALIZED_POINTS_NUMBER], clickedYaw[NORMALIZED_POINTS_NUMBER];
            idealYaw[0] = atan2(idealPoints[1].y - idealPoints[0].y, idealPoints[1].x - idealPoints[0].x);
            clickedYaw[0] = atan2(clickedPoints[1].y - clickedPoints[0].y, clickedPoints[1].x - clickedPoints[0].x);
            idealYaw[1] = atan2(idealPoints[2].y - idealPoints[1].y, idealPoints[2].x - idealPoints[1].x);
            clickedYaw[1] = atan2(clickedPoints[2].y - clickedPoints[1].y, clickedPoints[2].x - clickedPoints[1].x);
            idealYaw[2] = atan2(idealPoints[0].y - idealPoints[2].y, idealPoints[0].x - idealPoints[2].x);
            clickedYaw[2] = atan2(clickedPoints[0].y - clickedPoints[2].y, clickedPoints[0].x - clickedPoints[2].x);

            double yawDiff[NORMALIZED_POINTS_NUMBER], yawSum = 0.0;
            for(int index = 0; index < NORMALIZED_POINTS_NUMBER; ++index)
            {
                yawDiff[index] = idealYaw[index] - clickedYaw[index];
                if(yawDiff[index] > PI) yawDiff[index] -= (2 * PI);
                if(yawDiff[index] < -PI) yawDiff[index] += (2 * PI);
                ROS_INFO("nomalized info yaw_diff, index: %d, yaw: %lf", index, yawDiff[index]);
                yawSum += yawDiff[index];
            }
            double normalizedYaw = yawSum / (double)NORMALIZED_POINTS_NUMBER;
            for(int index = 0; index < NORMALIZED_POINTS_NUMBER; ++index)
            {
                if(fabs(normalizedYaw - yawDiff[index]) >= 0.1) ROS_ERROR("normalized failed!");
            }
            ROS_INFO("nomalized info: yaw: %lf", normalizedYaw);

            double xDiff[NORMALIZED_POINTS_NUMBER], yDiff[NORMALIZED_POINTS_NUMBER], xSum = 0, ySum = 0;
            geometry_msgs::Point RotatedPoint[NORMALIZED_POINTS_NUMBER];
            for(int index = 0; index < NORMALIZED_POINTS_NUMBER; ++index)
            {
                double clickedX = clickedPoints[index].x;
                double clickedY = clickedPoints[index].y;
                double radius = sqrt(clickedX * clickedX + clickedY * clickedY);
                double alpha = atan2(clickedY, clickedX);

                double rotatedX = radius * cos(alpha) * cos(normalizedYaw) - radius * sin(alpha) * sin(normalizedYaw);
                double rotatedY = radius * sin(alpha) * cos(normalizedYaw) + radius * cos(alpha) * sin(normalizedYaw);
                xDiff[index] = idealPoints[index].x - rotatedX;
                yDiff[index] = idealPoints[index].y - rotatedY;
                xSum += xDiff[index];
                ySum += yDiff[index];

                ROS_INFO("nomalized info: index: %d, x_diff: %lf, y_diff: %lf", index, xDiff[index], yDiff[index]);
            }
            double normalizedX = xSum / (double)NORMALIZED_POINTS_NUMBER;
            double normalizedY = ySum / (double)NORMALIZED_POINTS_NUMBER;
            for(int index = 0; index < NORMALIZED_POINTS_NUMBER; ++index)
            {
                if(fabs(normalizedX - xDiff[index]) >= 0.1) ROS_ERROR("normalized failed!");
                if(fabs(normalizedY - yDiff[index] >= 0.1)) ROS_ERROR("normalized failed!");
            }
            ROS_INFO("nomalized info: x: %lf, y: %lf", normalizedX, normalizedY);
            ROS_INFO("mapOffsetX = %lf, mapOffsetY = %lf, mapOffsetYaw = %lf", mapOffsetX, mapOffsetY, mapOffsetYaw);

            mapOffsetX = normalizedX;
            mapOffsetY = normalizedY;
            mapOffsetYaw = normalizedYaw;
        }
    }

    bool startNormalizeMapCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if(!normalizeStarted)
        {
            clickedPointSubscriber = nodeHandle.subscribe(clickedPointTopic, 3, &MultiGoalsNavigation::clickedPointCallBack, this);
            for(int index = 0; index < NORMALIZED_POINTS_NUMBER; ++index)
            {
                clickedPoints[index] = geometry_msgs::Point();
            }
            normalizedIndex = 0;

            res.success = true;
            res.message = "success"; 
            return true;
        }
        else
        {
            res.success = false;
            res.message = "fail"; 
            return false;
        }
    }

    void amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
    {
        if(!navigationStarted) return;
        ROS_INFO("amclPoseCallBack");


        double dx = amclMsg->pose.pose.position.x - publishedGoal.pose.position.x;
        double dy = amclMsg->pose.pose.position.y - publishedGoal.pose.position.y;
        double dist = sqrt(dx * dx + dy * dy);

        if(dist < goalRadius)
        {
            if(currentGoalIndex < optimalGoalsIndexVector.size() - 1)
            {
                currentGoalIndex++;
                publishGoal(currentGoalIndex);
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
        double initYaw = getYawFromPose(data->pose.pose) + mapOffsetYaw;
        if(initYaw >  PI) initYaw -= (2 * PI);
        if(initYaw < -PI) initYaw += (2 * PI);
        ROS_INFO("received init pose msg, started multi navigation");
        if(debugMode) ROS_INFO("init pose x: %lf, y: %lf, yaw: %lf", initPoint.x, initPoint.y, initYaw);

        ros::Duration waitDuration(1);
        waitDuration.sleep();
        std_srvs::Empty trigger;
        if(clearCostmapsClient.exists()) clearCostmapsClient.call(trigger);
        
        optimalGoalsIndexVector = getOptimalGoalsIndex(initYaw, goalsIndexVector);
        publishGoalMarker(goalsIndexVector);
        publishRoadMarker(optimalGoalsIndexVector);

        std::string str;
        str += std::string("multi goals navigation started, choose path: ");
        for(int index = 0; index < optimalGoalsIndexVector.size(); ++index)
        {
            str = str + std::to_string(optimalGoalsIndexVector[index] + 1) + std::string(" -> ");
        }
        str += std::string("end");
        ROS_INFO_STREAM(str);
 
        navigationStarted = true;

        currentGoalIndex = 1;
        publishGoal(currentGoalIndex);
    }

    void costmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& data)
    {
        double x = data->info.origin.position.x;
        double y = data->info.origin.position.y;
        double yaw = getYawFromPose(data->info.origin);
        ROS_INFO("global cost map x: %lf, y: %lf, yaw: %lf", x, y, yaw);
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
        std::vector<geometry_msgs::Point> pointVector;
        geometry_msgs::Point pt;

        std::vector<std::vector<double> > result = getMultiNumberVectorFromXMLRPC<double, 2>(xmlrpc, full_param_name);
        for(auto index = 0; index < result.size(); ++index)
        {
            pt.x = result[index][0];
            pt.y = result[index][1];
            pointVector.push_back(pt);
        }

        return pointVector;
    }

    std::vector<Dijkstra::LocationsRelation> getRelationVectorFromXMLRPC(XmlRpc::XmlRpcValue& xmlrpc, const std::string& full_param_name)
    {
        if (xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_FATAL("The para must be specified as list of lists on the parameter server, %s was specified as %s",
                        full_param_name.c_str(), std::string(xmlrpc).c_str());
            throw std::runtime_error("The param must be specified as list of lists");
        }

        std::vector<Dijkstra::LocationsRelation> relationVector;
        relationVector.clear();
        
        for (int i = 0; i < xmlrpc.size(); ++i)
        {
            XmlRpc::XmlRpcValue point = xmlrpc[ i ];
            if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                point.size() != 3)
            {
            ROS_FATAL("The (parameter %s) must be specified as list of lists on the parameter server, but this spec is not of that form.",
                        full_param_name.c_str());
            throw std::runtime_error("The param must be specified as list of lists on the parameter server, but this spec is not of that form");
            }

            Dijkstra::LocationsRelation lr;
            lr.from = getNumberFromXMLRPC<int>(point[ 0 ], full_param_name);
            lr.to = getNumberFromXMLRPC<int>(point[ 1 ], full_param_name);
            lr.punish = getNumberFromXMLRPC<double>(point[ 2 ], full_param_name);

            relationVector.push_back(lr);
        }
        return relationVector;
    }

    template<typename T, int multi>
    std::vector<std::vector<T> > getMultiNumberVectorFromXMLRPC(XmlRpc::XmlRpcValue& xmlrpc, const std::string& full_param_name)
    {
        if (xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_FATAL("The para must be specified as list of lists on the parameter server, %s was specified as %s",
                        full_param_name.c_str(), std::string(xmlrpc).c_str());
            throw std::runtime_error("The param must be specified as list of lists");
        }

        std::vector<std::vector<T> > numbersVector;
        numbersVector.clear();
        
        for (int i = 0; i < xmlrpc.size(); ++i)
        {
            XmlRpc::XmlRpcValue point = xmlrpc[ i ];
            if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                point.size() != multi)
            {
            ROS_FATAL("The (parameter %s) must be specified as list of lists on the parameter server, but this spec is not of that form.",
                        full_param_name.c_str());
            throw std::runtime_error("The param must be specified as list of lists on the parameter server, but this spec is not of that form");
            }

            std::vector<T> numbers;
            numbers.clear();
            for(int index = 0; index < multi; ++index)
            {
                numbers.push_back(getNumberFromXMLRPC<T>(point[ index ], full_param_name));
            }

            numbersVector.push_back(numbers);
        }
        return numbersVector;
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
