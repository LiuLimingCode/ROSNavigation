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

    struct Result
    {
        std::vector<int> bestPath;
        double cost = 0;
        double distance = 0;
        double angle = 0;
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
    std::vector<Result> run(int startIndex, double startAngle) const
    {
        std::vector<Result> result;
        std::priority_queue<HeapNode> heapNodeQueue;
        const int maxCost = 10000;
        bool reached[maxLocationsNum];

        for(int index = 0; index < maxLocationsNum; ++index)
        {
            reached[index] = false;
            result.push_back(Result());
            result[index].cost = maxCost;
        }
        result[startIndex].distance = 0;
        result[startIndex].angle = 0;
        result[startIndex].cost = 0;
        HeapNode startNode(startIndex, 0, 0, 0);
        startNode.path.clear();
        startNode.path.push_back(startIndex);
        result[startIndex].bestPath = startNode.path;
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
                    if(lastRelation->from == relation->to) relationAngle = PI * punishBackwards; // 如果是往返,直接等于PI 
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
                    if(fabs(relationAngle) > PI * 0.75) relationAngle *= punishBackwards; // 如果第一个点的转弯角度大于135,那么认为是倒退
                    else if(fabs(relationAngle) > PI * 0.25) relationAngle *= punishFristLargeBend; // 如果第一个弯转向过大,同样施加惩罚
                }
                
                double relationCost = (relation->dist() + relationAngle * punishBent) * relation->punish;
                if(result[relation->to].cost > result[relation->from].cost + relationCost)
                {
                    result[relation->to].cost = result[relation->from].cost + relationCost;
                    result[relation->to].distance = result[relation->from].distance + relation->dist();
                    result[relation->to].angle = result[relation->from].angle + relationAngle;
                    HeapNode nextNode(relation->to, result[relation->to].cost, result[relation->to].distance, result[relation->to].angle, relation, currentPath);
                    nextNode.path.push_back(relation->to);
                    result[relation->to].bestPath = nextNode.path;
                    heapNodeQueue.push(nextNode);
                }
            }
        }

        return result;
    }

    std::vector<LocationsRelation> locationsRelation; // 代表坐标点的关系
    std::vector<std::vector<int> > reachableIndexs; // 代表每个坐标点可到达的其他坐标点的序号
    double punishBent = 1.0;
    double punishBackwards = 1.0;
    double punishFristLargeBend = 1.0;
    int maxLocationsNum; // 坐标点的数量
};

class MultiGoalsNavigation
{
private:

    ros::NodeHandle nodeHandle;
    ros::Subscriber amclSubscriber, clickedPointSubscriber;
    ros::Publisher goalPublisher, markerPublisher;
    ros::ServiceServer normalizeServer, startNavigationServer, debugShowLocationsServer;
    ros::ServiceClient clearCostmapsClient;

    std::string mapFrame, clearCostmapsServer ,amclPoseTopic, goalTopic, markerTopic, clickedPointTopic;
    double goalRadius, goalExtension, punishBend, punishBackwards, punishFristLargeBend;
    bool debugMode, navigationStarted = false;
    visualization_msgs::Marker goalsMarker, roadMarker;

    XmlRpc::XmlRpcValue locationsXmlRpc, locationsRelationXmlRpc, goalsIndexXmlRpc, goalsStaticXmlRpc;

    std::vector<geometry_msgs::Point> locationVector; // 记录各个坐标点的坐标数据,从param中读取
    std::vector<Dijkstra::LocationsRelation> locationRelationVector; // 记录两两坐标点之间的关系,从param中读取
    std::vector<int> goalsIndexVector; // 记录目标在 locationVector 中的序号,从param中读取
    std::vector<bool> goalsStaticVector; // 根据该设置对目标点进行排序,从param中读取
    std::vector<int> optimalGoalsIndexVector; // 优化后得到的目标店序列

    geometry_msgs::Pose robotPose;
    int currentGoalIndex = 0; // 当前目标的序号
    const geometry_msgs::Point* currentGoal; // 当前坐标的数据
    geometry_msgs::PoseStamped publishedGoal; // currentGoal 经过归一化操作(平移、旋转)后,真正发布的坐标信息

    #define NORMALIZED_POINTS_NUMBER 4
    geometry_msgs::Point clickedPoints[NORMALIZED_POINTS_NUMBER]; // 进行归一化操作时,用于保存标定点数据
    int clickedPointsNumber = 0;
    int normalizedIndex = 0; // 记录当前归一化的区域
    bool normalizeStarted = false;
    geometry_msgs::Point (MultiGoalsNavigation::*normalizedFunction)(const geometry_msgs::Point *);

    Dijkstra dijkstra;

public:
    ~MultiGoalsNavigation() = default;
    MultiGoalsNavigation(ros::NodeHandle node)
    {
        nodeHandle = node;
        ros::NodeHandle _n("~");

        _n.param<std::string>("map_frame", mapFrame, "map");
        _n.param<std::string>("amcl_pose_topic", amclPoseTopic, "/amcl_pose");
        _n.param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");
        _n.param<std::string>("marker_topic", markerTopic, "visualization_marker");
        _n.param<std::string>("clicked_point_topic", clickedPointTopic, "/clicked_point");
        _n.param<std::string>("clear_costmaps_server", clearCostmapsServer, "/move_base/clear_costmaps");

        _n.param<double>("goal_radius", goalRadius, 1.0);
        _n.param<double>("goal_extension", goalExtension, goalRadius);
        _n.param<double>("punish_bend", punishBend, 1.0);
        _n.param<double>("punish_backwards", punishBackwards, 1.0);
        _n.param<double>("punish_fristlarge_bend", punishFristLargeBend, 1.0);
        dijkstra.punishBent = punishBend;
        dijkstra.punishBackwards = punishBackwards;
        dijkstra.punishFristLargeBend = punishFristLargeBend;

        _n.param<bool>("debug_mode", debugMode, false);

        _n.getParam("locations", locationsXmlRpc);
        _n.getParam("locations_relation", locationsRelationXmlRpc);
        _n.getParam("goals_id", goalsIndexXmlRpc);
        _n.getParam("goals_static", goalsStaticXmlRpc);

        initMarker();

        amclSubscriber = nodeHandle.subscribe(amclPoseTopic, 1, &MultiGoalsNavigation::amclPoseCallBack, this);
        clickedPointSubscriber.shutdown();
        goalPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>(goalTopic, 10);
        markerPublisher = nodeHandle.advertise<visualization_msgs::Marker>(markerTopic, 10);
        normalizeServer = nodeHandle.advertiseService("start_normalize_map", &MultiGoalsNavigation::startNormalizeMapCallBack, this);
        startNavigationServer = nodeHandle.advertiseService("start_navigation", &MultiGoalsNavigation::startNavigationCallBack, this);
        debugShowLocationsServer = nodeHandle.advertiseService("debug_show_locations", &MultiGoalsNavigation::debugShowLocationsCallBack, this);
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
                for(int temp = 0; temp < dijkstra.reachableIndexs[startIndex].size(); ++temp)
                {
                    const Dijkstra::LocationsRelation * relation = &dijkstra.locationsRelation[dijkstra.reachableIndexs[startIndex][temp]];
                    double yaw = atan2(relation->fromPoint->y - relation->toPoint->y, relation->fromPoint->x - relation->toPoint->x);
                    std::vector<Dijkstra::Result> dijkstraResult = dijkstra.run(startIndex, yaw);
                    ROS_INFO("#################################");
                    ROS_INFO("start point index: %d, start oriantation: %d -> %d, yaw: %lf", startIndex + 1, relation->to + 1, relation->from + 1, yaw);
                    for(int endIndex = 0; endIndex < locationVector.size(); ++endIndex)
                    {
                        std::string str = std::string("end point index: ") + std::to_string(endIndex + 1) + 
                                        std::string(" distance: ") + std::to_string(dijkstraResult[endIndex].distance) + 
                                        std::string(" angle: ") + std::to_string(dijkstraResult[endIndex].angle) + 
                                        std::string(" cost: ") + std::to_string(dijkstraResult[endIndex].cost);
                        str += std::string(" best path: ");
                        for(int index = 0; index < dijkstraResult[endIndex].bestPath.size(); ++index)
                        {
                            str = str + std::to_string(dijkstraResult[endIndex].bestPath[index] + 1) + std::string(" -> ");
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

            auto testOptimalResult = getOptimalGoalsIndex(yaw, goalsIndexVector);

            ROS_INFO("start point index: %d, start oriantation: %d -> %d, yaw: %lf", goalsIndexVector[0] + 1, relation->to + 1, relation->from + 1, yaw);
            std::string str = std::string(" distance: ") + std::to_string(testOptimalResult.distance) + 
                              std::string(" angle: ") + std::to_string(testOptimalResult.angle) + 
                              std::string(" cost: ") + std::to_string(testOptimalResult.cost);
            str += std::string(" best path: ");
            for(int index = 0; index < testOptimalResult.bestPath.size(); ++index)
            {
                str = str + std::to_string(testOptimalResult.bestPath[index] + 1) + std::string(" -> ");
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
        goalsMarker.pose.orientation.w = roadMarker.pose.orientation.w = 1;

        goalsMarker.id = 0;
        goalsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        goalsMarker.scale.x = goalRadius;
        goalsMarker.scale.y = goalRadius;
        goalsMarker.scale.y = 0.01;
        goalsMarker.color.r = 1.0;
        goalsMarker.color.g = 1.0;
        goalsMarker.color.b = 0.0;
        goalsMarker.color.a = 0.5;

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
            goalsMarker.points.push_back(locationVector[goalsIndex[index]]);
        }
        markerPublisher.publish(goalsMarker);
    }

    void publishRoadMarker(const std::vector<int> & optimalGoalsIndex)
    {
        roadMarker.points.clear();
        for(auto index = 0; index < optimalGoalsIndex.size(); ++index)
        {
            roadMarker.points.push_back(locationVector[optimalGoalsIndex[index]]);
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

    void publishGoal(int currentGoalIndex)
    {
        currentGoal = &locationVector[optimalGoalsIndexVector[currentGoalIndex]];
        ROS_INFO("current goal info (before rotation) index: %d, id: %d, x: %lf, y: %lf", currentGoalIndex, optimalGoalsIndexVector[currentGoalIndex] + 1, currentGoal->x, currentGoal->y);

        const geometry_msgs::Point * lastGoal = &locationVector[optimalGoalsIndexVector[currentGoalIndex - 1]];
        double currentAngel = atan2(currentGoal->y - lastGoal->y, currentGoal->x - lastGoal->x);

        double xPublished = currentGoal->x, yPublished = currentGoal->y, anglePublished = currentAngel;
        if(currentGoalIndex + 1 < optimalGoalsIndexVector.size() - 1)
        {
            const geometry_msgs::Point * futureGoal = &locationVector[optimalGoalsIndexVector[currentGoalIndex + 1]];
            anglePublished = atan2(futureGoal->y - currentGoal->y, futureGoal->x - currentGoal->x);
            xPublished += (cos(anglePublished) * goalExtension);
            yPublished += (sin(anglePublished) * goalExtension);
        }

        publishedGoal = geometry_msgs::PoseStamped();
        tf::Quaternion q = tf::createQuaternionFromYaw(anglePublished);

        publishedGoal.header.frame_id = mapFrame;
        publishedGoal.header.stamp = ros::Time::now();
        publishedGoal.pose.position.x = xPublished;
        publishedGoal.pose.position.y = yPublished;
        publishedGoal.pose.position.z = 0;
        publishedGoal.pose.orientation.w = q.w();
        publishedGoal.pose.orientation.x = q.x();
        publishedGoal.pose.orientation.y = q.y();
        publishedGoal.pose.orientation.z = q.z();
        if(debugMode) ROS_INFO("current goal info (after rotation) index: %d, id: %d, x: %lf, y: %lf", currentGoalIndex, optimalGoalsIndexVector[currentGoalIndex] + 1, xPublished, yPublished);

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

    Dijkstra::Result chooceBestPath(int startIndex, double startYaw, int stopIndex, const std::vector<int>& goalsIndex) const
    {
        std::vector<std::vector<int> > goalsPermutation;
        std::vector<int> tempGoalsIndex = goalsIndex;
        numberPermutation(tempGoalsIndex, 0, tempGoalsIndex.size(), &goalsPermutation);
        Dijkstra::Result result;

        result.cost = 1000000;
        int minIndex = 0;
        for(int index = 0; index < goalsPermutation.size(); ++index) // 对所有可能性进行遍历
        {
            std::vector<int> goalsSelected = goalsPermutation[index]; // 得到排列组合后的序列,并加上起点终点信息
            goalsSelected.insert(goalsSelected.begin(), startIndex);
            goalsSelected.push_back(stopIndex);

            double costSum = 0;
            double distanceSum = 0;
            double angleSum = 0;
            std::vector<int> pathSum, lastBestPath;
            for(int alpha = 1; alpha < goalsSelected.size(); ++alpha)
            {
                const int begin = goalsSelected[alpha - 1];
                const int end = goalsSelected[alpha];
                double oriantation = startYaw;
                if(alpha >= 2)
                {
                    const geometry_msgs::Point * fromPoint = &locationVector[lastBestPath[lastBestPath.size() - 2]];
                    const geometry_msgs::Point * toPoint = &locationVector[lastBestPath[lastBestPath.size() - 1]];
                    oriantation = atan2(toPoint->y - fromPoint->y, toPoint->x - fromPoint->x);
                }

                auto dijkstraResult = dijkstra.run(begin, oriantation);

                lastBestPath = dijkstraResult[end].bestPath;
                costSum += dijkstraResult[end].cost;
                distanceSum += dijkstraResult[end].distance;
                angleSum += dijkstraResult[end].angle;
                for(int beta = 1; beta < dijkstraResult[end].bestPath.size(); ++beta)
                {
                    pathSum.push_back(dijkstraResult[end].bestPath[beta]);
                }
            }

            if(costSum < result.cost)
            {
                minIndex = index;
                result.cost = costSum;
                result.angle = angleSum;
                result.distance = distanceSum;
                result.bestPath = pathSum;
            }
        }

        result.bestPath.insert(result.bestPath.begin(), startIndex);
        return result;
    }

    Dijkstra::Result getOptimalGoalsIndex(const double startYaw, const std::vector<int>& goalsIndex) const
    {
        std::vector<int> goalsNeedOptimize;
        Dijkstra::Result result;

        result.bestPath.push_back(goalsIndex[0]); // 第一个点一定是固定的起点
        for(int index = 1; index < goalsIndexVector.size(); ++index)
        {
            if(!goalsStaticVector[index])
            {
                goalsNeedOptimize.push_back(goalsIndexVector[index]);
            }
            else
            {
                int startIndex = result.bestPath[result.bestPath.size() - 1]; // 优化队列中最后一个值当做起点
                int endIndex = goalsIndexVector[index]; // 当前值当做终点
                double oriantation = startYaw;
                if(result.bestPath.size() >= 2)
                {
                    const geometry_msgs::Point * fromPoint = &locationVector[result.bestPath[result.bestPath.size() - 2]];
                    const geometry_msgs::Point * toPoint = &locationVector[result.bestPath[result.bestPath.size() - 1]];
                    oriantation = atan2(toPoint->y - fromPoint->y, toPoint->x - fromPoint->x);
                }
                auto resultSegment = chooceBestPath(startIndex, oriantation, endIndex, goalsNeedOptimize);

                result.cost += resultSegment.cost;
                result.angle += resultSegment.angle;
                result.distance += resultSegment.distance;
                for(int temp = 1; temp < resultSegment.bestPath.size(); ++temp) result.bestPath.push_back(resultSegment.bestPath[temp]);

                goalsNeedOptimize.clear();
            }
        }

        return result;
    }

    geometry_msgs::Point normalizedLocationWithTowPoints(const geometry_msgs::Point * points)
    {
        geometry_msgs::Point result;
        result.x = (points[0].x + points[1].x) / 2.0;
        result.y = (points[0].y + points[1].y) / 2.0;
        
        return result;
    }

    geometry_msgs::Point normalizedLocationWithThreePoints(const geometry_msgs::Point * points)
    {
        geometry_msgs::Point result;
        if(points[0].x == points[1].x)
        {
            result.x = (points[0].x + points[2].x) / 2.0;
            result.y = (points[0].y + points[1].y) / 2.0;
        }
        else if(points[0].y == points[1].y)
        {
            result.x = (points[0].x + points[1].x) / 2.0;
            result.y = (points[0].y + points[2].y) / 2.0;
        }
        else
        {
            geometry_msgs::Point middlePoint, endPoint;
            middlePoint.x = (points[0].x + points[1].x) / 2.0;
            middlePoint.y = (points[0].y + points[1].y) / 2.0;

            double yaw = atan2(points[0].y - points[1].y, points[0].x - points[1].x);
            double k1 = tan(yaw);
            double k2 = tan(yaw - PI / 2.0);
            double b1 = points[2].y - k1 * points[2].x; // y = kx + b
            double b2 = middlePoint.y - k2 * middlePoint.x;
            endPoint.x = (b2 - b1) / (k1 - k2);
            endPoint.y = (k1 * b2 - k2 * b1) / (k1 - k2);

            result.x = (middlePoint.x + endPoint.x) / 2.0;
            result.y = (middlePoint.y + endPoint.y) / 2.0;
        }

        return result;
    }

    geometry_msgs::Point normalizedLocationWithFourPoints(const geometry_msgs::Point * points)
    {
        geometry_msgs::Point result;
        double k1 = 0, k2 = 0, b1 = 0, b2 = 0;
        bool f1 = false, f2 = false;
        if(points[0].x == points[1].x)
        {
            f1 = true;
            b1 = points[0].x;
        }
        else
        {
            k1 = (points[0].y - points[1].y) / (points[0].x - points[1].x);
            b1 = points[0].y - k1 * points[0].x;
        }
        
        if(points[2].x == points[3].x)
        {
            f2 = true;
            b2 = points[2].x;
        }
        else
        {
            k2 = (points[2].y - points[3].y) / (points[2].x - points[3].x);
            b2 = points[2].y - k2 * points[2].x;
        }

        if(f1)
        {
            result.x = b1;
            result.y = k2 * b1 + b2;
        }
        if(f2)
        {
            result.x = b2;
            result.y = k1 * b2 + b1;
        }
        else
        {
            result.x = (b2 - b1) / (k1 - k2);
            result.y = (k1 * b2 - k2 * b1) / (k1 - k2);
        }

        return result;
    }

    void clickedPointCallBack(const geometry_msgs::PointStamped::ConstPtr& point)
    {
        if(!normalizeStarted) return;

        clickedPoints[clickedPointsNumber] = point->point;
        ROS_INFO("clicked info: points, index: %d, x: %lf, y: %lf", clickedPointsNumber, point->point.x, point->point.y);
        ++clickedPointsNumber;
        int clickedPointsNeed = 0;

        if(normalizedIndex == 0 || normalizedIndex == 3 ||  normalizedIndex == 5 || normalizedIndex == 8)
        {
            clickedPointsNeed = 2;
            normalizedFunction = &MultiGoalsNavigation::normalizedLocationWithTowPoints;
        }
        else if(normalizedIndex == 1 || normalizedIndex == 2 ||normalizedIndex == 4 || normalizedIndex == 6 || normalizedIndex == 7 || normalizedIndex == 10 || normalizedIndex == 11)
        {
            clickedPointsNeed = 3;
            normalizedFunction = &MultiGoalsNavigation::normalizedLocationWithThreePoints;
        }
        else if(normalizedIndex == 9)
        {
            clickedPointsNeed = 4;
            normalizedFunction = &MultiGoalsNavigation::normalizedLocationWithFourPoints;
        }
        ROS_INFO("clickedPointsNeed: %d, clickedPointsNumber: %d", clickedPointsNeed, clickedPointsNumber);

        if(clickedPointsNumber == clickedPointsNeed)
        {
            clickedPointsNumber = 0;
            locationVector[normalizedIndex] = (this->*normalizedFunction)(clickedPoints);
            ROS_INFO("normalization info: index = %d, x: %lf, %lf", normalizedIndex + 1, locationVector[normalizedIndex].x, locationVector[normalizedIndex].y);
            std_srvs::Trigger trigger;
            debugShowLocationsCallBack(trigger.request, trigger.response);
            ++normalizedIndex;
            if(normalizedIndex == locationVector.size())
            {
                clickedPointSubscriber.shutdown();
                normalizeStarted = false;
            }
        }
    }

    bool debugShowLocationsCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::vector<int> allGoalsIndex;
        for(int index = 0; index < locationVector.size(); ++index)
        {
            allGoalsIndex.push_back(index);
        }
        publishGoalMarker(allGoalsIndex);
        res.success = true;
        return true;
    }

    bool startNormalizeMapCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if(!normalizeStarted)
        {
            clickedPointSubscriber = nodeHandle.subscribe(clickedPointTopic, NORMALIZED_POINTS_NUMBER, &MultiGoalsNavigation::clickedPointCallBack, this);
            ROS_INFO("start normalization");
            for(int index = 0; index < NORMALIZED_POINTS_NUMBER; ++index)
            {
                clickedPoints[index] = geometry_msgs::Point();
            }
            std_srvs::Trigger trigger;
            debugShowLocationsCallBack(trigger.request, trigger.response);
            normalizedIndex = 0;
            clickedPointsNumber = 0;
            normalizeStarted = true;

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

    bool startNavigationCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        double initYaw = getYawFromPose(robotPose);
        ROS_INFO("received init pose msg, started multi navigation");
        if(debugMode) ROS_INFO("init pose x: %lf, y: %lf, yaw: %lf", robotPose.position.x, robotPose.position.y, initYaw);

        std_srvs::Empty trigger;
        if(clearCostmapsClient.exists()) clearCostmapsClient.call(trigger);
        ros::Duration waitDuration(1.0);
        waitDuration.sleep();
        
        auto OptimalResult = getOptimalGoalsIndex(initYaw, goalsIndexVector);
        optimalGoalsIndexVector = OptimalResult.bestPath;
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
        res.success = true;
        res.message = "success";
        return true;
    }

    void amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
    {
        robotPose = amclMsg->pose.pose;
        if(!navigationStarted) return;

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
