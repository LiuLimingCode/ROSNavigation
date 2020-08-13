#include "../include/art_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int16.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

std::string ackermann_topic;
std::string Motor_pid_Out_std_msgs_topic;

uint16_t angle_pwm_max_us=2500;     //右打死
uint16_t angle_pwm_mid_us=1850;     //居中(一般无需设置，自动换算)
uint16_t angle_pwm_min_us=1200;     //左打死


uint16_t speed_pwm_max_us=2500;     //向前最高速度
uint16_t speed_pwm_mid_us=1500;     //静止速度
uint16_t speed_pwm_min_us=500;     //向后最高速度
uint16_t Car_Stop_Flag = 1;

double speed_max_m_s=8.0;           //理论最高速度(国际单位制m/s)
double angle_max_rad=0.6108652382;  //理论最大打角(国际单位制rad）

uint16_t angle_out=angle_pwm_mid_us;
uint16_t speed_out=speed_pwm_min_us;

void AckermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& Ackermann_set)
{
    double angle_set=Ackermann_set->drive.steering_angle;
    double speed_set=Ackermann_set->drive.speed;

    //车模保护停止
     if(Car_Stop_Flag == 1)
         speed_set = 0;

    angle_out = (0.5-angle_set*(1/(2*angle_max_rad)))*(angle_pwm_max_us-angle_pwm_min_us)+angle_pwm_min_us;
    //根据公式->{（angle-pwm_min）/（pwm_max+pwm_min）}*(2*angle_max_rad)-angle_max_rad=Ackermann.drive.steering_angle得出，将转向的角度值转换为高电平时间（us）
    // speed_out = (speed_set/speed_max_m_s)*(speed_pwm_max_us-speed_pwm_mid_us)+speed_pwm_mid_us;
    //根据公式->{（speed-pwm_min）/（pwm_max+pwm_min）}*speed_max_m_s得出，将设定的速度（m/s）转换为电机pwm的高电平时间
    speed_out = speed_set * 100 + 32767;
    //舵机限幅
    if(angle_out>angle_pwm_max_us)angle_out=angle_pwm_max_us;
    if(angle_out<angle_pwm_min_us)angle_out=angle_pwm_min_us;

    send_cmd(uint16_t(speed_out),uint16_t(angle_out));
}

void Car_Stop_Callback(const std_msgs::Int16ConstPtr &Stop_Flag)
{
    if(Stop_Flag->data == 1)
        Car_Stop_Flag = 1;
    if(Stop_Flag->data == 2)
        Car_Stop_Flag = 0;
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(115200,data);
    send_cmd(uint16_t(32767),uint16_t(angle_pwm_mid_us));
    ros::init(argc, argv, "art_driver");
    ros::NodeHandle n;

    n.param<std::string>("ackermann_cmd_topic", ackermann_topic, "/cmd_ackermann");
    n.param<std::string>("topic_from_controller", Motor_pid_Out_std_msgs_topic, "Motor_pid_Out_std_msgs");

    ros::Subscriber sub_ack_angle = n.subscribe(ackermann_topic, 1, AckermannCallback);

    // ros::Subscriber sub_pid_ctrl_speed=n.subscribe(Motor_pid_Out_std_msgs_topic,1,MOTOR_PID_OUT_Callback);    //速度闭环

    ros::Subscriber sub_Stop_car = n.subscribe("Car_Stop",1,Car_Stop_Callback);
    ros::spin();
    send_cmd(uint16_t(32767),uint16_t(angle_pwm_mid_us));
}