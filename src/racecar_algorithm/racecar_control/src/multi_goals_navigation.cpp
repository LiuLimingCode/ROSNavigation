#include <iostream>
#include <string>
#include <queue>
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

    class Dijskra
    {
    public:

        struct LocationsRelation
        {
            int from;
            int to;
            const geometry_msgs::Point * fromPoint;
            const geometry_msgs::Point * toPoint;

            LocationsRelation (int fromIndex = 0, int toIndex = 0, const geometry_msgs::Point * pFromPoint = nullptr, const geometry_msgs::Point * pToPoint = nullptr) :
            from(fromIndex), to(toIndex), fromPoint(pFromPoint), toPoint(pToPoint) {}
            double dist(void) const
            {
                double dx = fromPoint->x - toPoint->x;
                double dy = fromPoint->y - toPoint->y;
                return sqrt(dx * dx + dy * dy);
            }
            double angle(void) const { return atan2(toPoint->y - fromPoint->y, toPoint->x - fromPoint->x); }
            double cost(const LocationsRelation* lastRelation = nullptr) const
            {
                return dist();
            }
        };

        struct HeapNode // Dijkstra算法用到的优先队列的节点
        {
            int cost, index;
            const LocationsRelation* relation;
            std::vector<int> path;
            HeapNode(int i, double c, const LocationsRelation* r = nullptr, std::vector<int> p = std::vector<int>()) : cost(c), index(i), relation(r), path(p) {}
            bool operator < (const HeapNode &rhs) const
            {
                return cost > rhs.cost;
            }
        };

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

        void run(int startIndex, double * distance, std::vector<int> * bestPath)
        {
            std::priority_queue<HeapNode> heapNodeQueue;
            const int maxCost = 10000;
            bool reached[maxLocationsNum];

            for(int index = 0; index < maxLocationsNum; ++index)
            {
                reached[index] = false;
                distance[index] = maxCost;
            }
            distance[startIndex] = 0;
            HeapNode startNode(startIndex, 0);
            startNode.path.clear();
            startNode.path.push_back(startIndex);
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
                    
                    if(distance[relation->to] > distance[relation->from] + relation->cost(lastRelation))
                    {
                        distance[relation->to] = distance[relation->from] + relation->cost(lastRelation);
                        HeapNode nextNode(relation->to, distance[relation->to], relation, currentPath);
                        nextNode.path.push_back(relation->to);
                        bestPath[relation->to] = nextNode.path;
                        heapNodeQueue.push(nextNode);
                    }
                }
            }
        }

    private:

    std::vector<LocationsRelation> locationsRelation; // 代表坐标点的关系
    std::vector<std::vector<int> > reachableIndexs; // 代表每个坐标点的index可到达的其他坐标点index
    int maxLocationsNum;

    };

    ros::NodeHandle nodeHandle;
    ros::Subscriber amclSubscriber, goalSubscriber;
    ros::Publisher goalPublisher;

    std::string mapFrame, amclPoseTopic, initPoseTopic, goalTopic;

    double goalRadius, mapOffsetX, mapOffsetY;

    bool debugMode, navigationStarted;

    XmlRpc::XmlRpcValue locationsXmlRpc, locationsRelationXmlRpc, goalsIndexXmlRpc, goalsStaticXmlRpc;

    std::vector<geometry_msgs::Point> locationVector;
    std::vector<Dijskra::LocationsRelation> locationRelationVector;
    std::vector<int> goalsIndexVector;
    std::vector<bool> goalsStaticVector;

    int currentGoalIndex = 0;
    const geometry_msgs::Point * currentGoal;

    Dijskra dijskra;
    std::vector<double *> distance;
    std::vector<std::vector<int> *> bestPath;

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
        _n.getParam("locations_relation", locationsRelationXmlRpc);
        _n.getParam("goals_id", goalsIndexXmlRpc);
        _n.getParam("goals_static", goalsStaticXmlRpc);

        amclSubscriber = nodeHandle.subscribe(amclPoseTopic, 1, &MultiGoalsNavigation::amclPoseCallBack, this);
        goalSubscriber = nodeHandle.subscribe(initPoseTopic, 1, &MultiGoalsNavigation::initPoseoalCallBack, this);
        goalPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>(goalTopic, 10);

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

        if(debugMode) ROS_INFO("locations relation:");
        for(auto index = 0; index < locationRelationVector.size(); ++index)
        {
            int formIndex = --locationRelationVector[index].from;
            int toIndex = --locationRelationVector[index].to;

            checkCondition(formIndex > locationVector.size() - 1, "fatal error: locations_relation param is wrong");
            locationRelationVector[index].fromPoint = &locationVector[formIndex];
            checkCondition(toIndex > locationVector.size() - 1, "fatal error: locations_relation param is wrong");
            locationRelationVector[index].toPoint = &locationVector[toIndex];
            if(debugMode) ROS_INFO("form: %d, to: %d", formIndex + 1, toIndex + 1);
        }

        if(debugMode) ROS_INFO("goals info:");
        for(auto index = 0; index < goalsIndexVector.size(); ++index)
        {
            goalsIndexVector[index] -= 1;
            checkCondition(goalsIndexVector[index] > locationVector.size() - 1, "fatal error: some elements in goals_id array param is large than the size of locations array param");
            if(debugMode) ROS_INFO("index: %d, id: %d, static: %d, goal.x: %lf, goal.y: %lf", index, goalsIndexVector[index] + 1, (int)goalsStaticVector[index], 
                locationVector[goalsIndexVector[index]].x, locationVector[goalsIndexVector[index]].y);
        }

        if(debugMode) ROS_INFO("dijskra results:");
        dijskra.init(locationVector.size(), locationRelationVector);

        for(int startIndex = 0; startIndex < locationVector.size(); ++startIndex)
        {
            distance.push_back(new double[locationVector.size()]);
            bestPath.push_back(new std::vector<int>[locationVector.size()]);
            dijskra.run(startIndex, distance[startIndex], bestPath[startIndex]);

            if(debugMode)
            {
                ROS_INFO("#################################");
                ROS_INFO("start point index: %d", startIndex + 1);
                for(int endIndex = 0; endIndex < locationVector.size(); ++endIndex)
                {
                    std::string str = std::string("end point index: ") + std::to_string(endIndex + 1) + std::string(" distance: ") + std::to_string(distance[startIndex][endIndex]);
                    str += std::string(" best path: ");
                    for(int index = 0; index < bestPath[startIndex][endIndex].size(); ++index)
                    {
                        str = str + std::to_string(bestPath[startIndex][endIndex][index] + 1) + std::string(" -> ");
                    }
                    str += std::string("end");
                    ROS_INFO_STREAM(str);
                }
            }
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
            if(currentGoalIndex < goalsIndexVector.size() - 1)
            {
                currentGoalIndex++;
                currentGoal = &locationVector[goalsIndexVector[currentGoalIndex]];
                if(debugMode) ROS_INFO("current goal index: %d, id: %d, x: %lf, y: %lf", currentGoalIndex, goalsIndexVector[currentGoalIndex] + 1, currentGoal->x, currentGoal->y);
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
        currentGoal = &locationVector[goalsIndexVector[currentGoalIndex]];
        if(debugMode) ROS_INFO("current goal index: %d, id: %d, x: %lf, y: %lf", currentGoalIndex, goalsIndexVector[currentGoalIndex] + 1, currentGoal->x, currentGoal->y);
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

    std::vector<Dijskra::LocationsRelation> getRelationVectorFromXMLRPC(XmlRpc::XmlRpcValue& xmlrpc, const std::string& full_param_name)
    {
        std::vector<Dijskra::LocationsRelation> relationVector;
        Dijskra::LocationsRelation lr;

        std::vector<std::vector<int> > result = getMultiNumberVectorFromXMLRPC<int, 2>(xmlrpc, full_param_name);
        for(auto index = 0; index < result.size(); ++index)
        {
            lr.from = result[index][0];
            lr.to = result[index][1];
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
