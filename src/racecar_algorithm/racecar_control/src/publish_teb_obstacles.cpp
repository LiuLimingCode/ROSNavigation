#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/PointStamped.h"
#include "costmap_converter/ObstacleMsg.h"
#include "costmap_converter/ObstacleArrayMsg.h"
#include "visualization_msgs/Marker.h"

ros::NodeHandle* node;

ros::Publisher obstaclesPublisher, markerPublisher;
ros::Subscriber clickedPointSubscriber;
ros::ServiceServer subscribeClickedPoint;

visualization_msgs::Marker obstaclesMarker;

int currentIndex = 0;
double obstaclesRadius;

void publishObstacleMarker(const geometry_msgs::Point point)
{
    obstaclesMarker.points.push_back(point);
    markerPublisher.publish(obstaclesMarker);
}

void publishObstacle(const geometry_msgs::Point point)
{
    costmap_converter::ObstacleArrayMsg msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    msg.obstacles.push_back(costmap_converter::ObstacleMsg());
    msg.obstacles[0].id = currentIndex++;
    msg.obstacles[0].polygon.points.push_back(geometry_msgs::Point32());
    msg.obstacles[0].polygon.points[0].x = point.x;
    msg.obstacles[0].polygon.points[0].y = point.y;
    msg.obstacles[0].polygon.points[0].z = point.z;
    msg.obstacles[0].radius = obstaclesRadius;

    publishObstacleMarker(point);
    ROS_INFO("publish teb obstacles, x: %lf, y: %lf", point.x, point.y);

    obstaclesPublisher.publish(msg);
}

template<typename T>
T getNumberFromXMLRPC(XmlRpc::XmlRpcValue& xmlrpc)
{
    // Make sure that the value we're looking at is either a double or an int.
    if (xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    {
        std::string& value_string = xmlrpc;
        throw std::runtime_error("Values in the param must be numbers");
    }
    T number;
    if(xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeInt) number = (int)(xmlrpc);
    else if(xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeDouble) number = (double)(xmlrpc);
    else if(xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeBoolean) number = (bool)(xmlrpc);

    return number;
}

void publishObstacleWithYaml(XmlRpc::XmlRpcValue& xmlrpc)
{
    if (xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        throw std::runtime_error("The param must be specified as list of lists");
    }
    
    for (int i = 0; i < xmlrpc.size(); ++i)
    {
        XmlRpc::XmlRpcValue segment = xmlrpc[ i ];
        if (segment.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            segment.size() != 2)
        {
            throw std::runtime_error("The param must be specified as list of lists on the parameter server, but this spec is not of that form");
        }

        geometry_msgs::Point point;
        double number[2];
        for(int index = 0; index < 2; ++index)
        {
            number[index] = getNumberFromXMLRPC<double>(segment[ index ]);
        }
        point.x = number[0];
        point.y = number[1];
        publishObstacle(point);
    }
}

void clickedPointCallBack(const geometry_msgs::PointStamped::ConstPtr & data)
{
    publishObstacle(data->point);
}

bool subscribeClickedPointCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if(req.data)
    {
        clickedPointSubscriber = node->subscribe("/clicked_point", 1, clickedPointCallBack);
    }
    else
    {
        clickedPointSubscriber.shutdown();
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_teb_obstacles_node");
    node = new ros::NodeHandle("~");

    XmlRpc::XmlRpcValue obstaclesXmlRpc;
    node->getParam("obstacles", obstaclesXmlRpc);
    node->param<double>("obstacles_radius", obstaclesRadius, 0.1);

    obstaclesPublisher = node->advertise<costmap_converter::ObstacleArrayMsg>("/move_base/TebLocalPlannerROS/obstacles", 10, true);
    markerPublisher = node->advertise<visualization_msgs::Marker>("/publish_teb_obstacles/marker", 10, true);
    subscribeClickedPoint = node->advertiseService("subscribe_clicked_point", subscribeClickedPointCallBack);

    obstaclesMarker.header.frame_id = "map";
    obstaclesMarker.ns = "publish_teb_obstacles";
    obstaclesMarker.action = visualization_msgs::Marker::ADD;
    obstaclesMarker.pose.orientation.w = 1;

    obstaclesMarker.id = 0;
    obstaclesMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    obstaclesMarker.scale.x = obstaclesRadius;
    obstaclesMarker.scale.y = obstaclesRadius;
    obstaclesMarker.scale.z = 0.1;
    obstaclesMarker.color.r = 0.2;
    obstaclesMarker.color.g = 0.2;
    obstaclesMarker.color.b = 0.2;
    obstaclesMarker.color.a = 0.5;

    try
    {
        publishObstacleWithYaml(obstaclesXmlRpc);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }

    ros::spin();

    return 0;
}