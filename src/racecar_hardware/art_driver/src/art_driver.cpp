//
// Created by Steven Zhang on 18-12-14.
// art racecar
//

#include "../include/art_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

std::string ackermann_topic;

uint16_t angle_pwm_max_us=2500;     //右打死
uint16_t angle_pwm_mid_us=1696;     //居中(一般无需设置，自动换算)
uint16_t angle_pwm_min_us=1090;     //左打死


uint16_t speed_pwm_max_us=2000;     //向前最高速度
uint16_t speed_pwm_min_us=1500;     //静止速度

double speed_max_m_s=8.0;           //理论最高速度(国际单位制m/s)
double angle_max_rad=0.7853981634;  //理论最大打角(国际单位制rad）

void AckermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& Ackermann)
{
    uint16_t angle_out;
    uint16_t speed_out;
    double speed_set=Ackermann->drive.speed;
    double angle_set=Ackermann->drive.steering_angle;

    if(speed_set < 0)
        speed_set=-2.0;

    angle_out = (0.5-angle_set*(1/(2*angle_max_rad)))*(angle_pwm_max_us-angle_pwm_min_us)+angle_pwm_min_us;
    //根据公式->{（angle-pwm_min）/（pwm_max+pwm_min）}*(2*angle_max_rad)-angle_max_rad=Ackermann.drive.steering_angle得出，将转向的角度值转换为高电平时间（us）
    speed_out = (speed_set/speed_max_m_s)*(speed_pwm_max_us-speed_pwm_min_us)+speed_pwm_min_us;
    //根据公式->{（speed-pwm_min）/（pwm_max+pwm_min）}*speed_max_m_s得出，将设定的速度（m/s）转换为电机pwm的高电平时间

    ROS_INFO("speed= %d", speed_out);
    ROS_INFO("angle= %d", angle_out);
    send_cmd(uint16_t(speed_out),uint16_t(angle_out));
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