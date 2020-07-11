#include <ros/ros.h>
#include "path_pursuit.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_pursuit_node");

    PathPursuit pathPursuit(ros::NodeHandle("~"));

    ros::spin();

    return(0);
}