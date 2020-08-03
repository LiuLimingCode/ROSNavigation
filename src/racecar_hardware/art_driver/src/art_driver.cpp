//
// Created by Steven Zhang on 18-12-14.
// art racecar
//

#include "../include/art_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

std::string ackermann_topic;

uint16_t angle_pwm_max_us=2303;
uint16_t angle_pwm_min_us=1090;
uint16_t speed_pwm_max_us=1600;
uint16_t speed_pwm_min_us=1350;

double speed_max_m_s=3.0;

double PI=3.14159265354;

void AckermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& Ackermann)
{
    uint16_t angle;
    uint16_t speed;
    angle = (Ackermann->drive.steering_angle+PI/4.0)*(2/PI)*(angle_pwm_max_us-angle_pwm_min_us)+angle_pwm_min_us;
    //根据公式->{（angle-pwm_min）/（pwm_max+pwm_min）}*(PI/2.0)-(PI/4.0)=Ackermann.drive.steering_angle得出，将转向的角度值转换为高电平时间（us）
    speed = (Ackermann->drive.speed/speed_max_m_s)*(speed_pwm_max_us-speed_pwm_min_us)+speed_pwm_min_us;
    //根据公式->{（speed-pwm_min）/（pwm_max+pwm_min）}*speed_max_m_s得出，将设定的速度（m/s）转换为电机pwm的高电平时间
    ROS_INFO("speed= %d", speed);
    ROS_INFO("angle= %d", angle);
    send_cmd(uint16_t(speed),uint16_t(angle));
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(115200,data);
    ros::init(argc, argv, "art_driver");

	ros::NodeHandle n;
    
	n.param<std::string>("ackermann_cmd_topic", ackermann_topic, "/cmd_ackermann");

	ros::Subscriber sub = n.subscribe(ackermann_topic, 1, AckermannCallback);
    
    ros::spin();
}