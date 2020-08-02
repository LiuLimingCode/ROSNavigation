//
// Created by Steven Zhang on 18-12-14.
// art racecar
//

#include "../include/art_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

std::string ackermann_cmd_topic;

void AckermannCallback(const ackermann_msgs::AckermannDriveStamped& Ackermann)
{
    double angle;
    
    
    
    angle = 2500.0 - Ackermann.drive.steering_angle * 2000.0 / 180.0;
    ROS_INFO("Ackermann.drive.speed= %f", Ackermann.drive.speed);
    ROS_INFO("Ackermann.drive.steering_angle= %f", Ackermann.drive.steering_angle);
    ROS_INFO("angle= %d",uint16_t(angle));
    send_cmd(uint16_t(Ackermann.drive.speed),uint16_t(angle));
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(115200,data);
    ros::init(argc, argv, "art_driver");

	ros::NodeHandle node("~");
    
	node.param<std::string>("ackermann_cmd_topic", ackermann_cmd_topic, "/cmd_ackermann");

	ros::Subscriber sub = node.subscribe(ackermann_cmd_topic, 1, AckermannCallback);

    ros::spin();
	return(0);

}