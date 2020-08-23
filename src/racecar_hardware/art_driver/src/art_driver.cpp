#include "../include/art_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int16.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

std::string ackermann_topic;

//舵机参数
double Angle_P = 0;
double Angle_D = 0;
double angle_max_rad=0.6108652382;  //理论最大打角(国际单位制rad）
uint16_t angle_pwm_max_us=2373;     //右打死
uint16_t angle_pwm_mid_us=1918;     //居中(一般无需设置，自动换算)
uint16_t angle_pwm_min_us=1464;     //左打死
uint16_t angle_out=angle_pwm_mid_us;


//速度参数
uint16_t speed_send_zero = 32767;  //串口发送的速度零点
int16_t Car_Stop_Flag = 1;
uint16_t speed_out = speed_send_zero;


double PID_Local(double Aid,double Measure,double P,double D)
{
    double PID_OUT = 0;
    double Error = 0;
    double D_Data = 0;
    static double Error_Last = 0;
    static double D_Data_Last = 0;
    Error = Aid - Measure;
    D_Data = Error - Error_Last;
    PID_OUT = P * Error
             +D * (1.0*D_Data + 0.0*D_Data_Last);
    Error_Last = Error;
    D_Data_Last = D_Data;
    return PID_OUT;
}


void AckermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& Ackermann_set)
{
    double angle_set=Ackermann_set->drive.steering_angle;
    double angle_ctrl_out = 0;
    double speed_set=Ackermann_set->drive.speed;

    // static uint16_t Timer_Stop = 0 , Timer_Wait = 0;
    // static uint8_t flag = 0;
    // if((flag == 0) && (fabs(angle_set) >= 3))
    // {
    //     flag = 1;
    // }
    // if(flag == 1)
    // {
    //     Timer_Stop++;
    //     if(speed_set > 0) speed_set = 0.5;
    //     else if(speed_set < 0) speed_set = -0.5;
    //     else if(speed_set == 0) speed_set = 0;
    // }
    // if((flag == 1) && (Timer_Stop >= 3) )
    // {
    //     flag = 2;
    //     Timer_Stop = 0;
    // }
    // if(flag == 2)
    // {
    //     Timer_Wait++;
    // }
    // if((flag == 2) && (Timer_Wait >= 6))
    // {
    //     flag = 0;
    //     Timer_Wait = 0;
    // }

    //车模保护停止
     if(Car_Stop_Flag == 1)
         speed_set = 0;


    angle_ctrl_out = - PID_Local(0,angle_set,Angle_P,Angle_D); 
     //舵机限幅 , 判断浮点数大小应判断差值小于一个很小的数
    if((angle_ctrl_out - angle_max_rad) > 0.0000001) angle_ctrl_out = angle_max_rad;
    if((angle_ctrl_out + angle_max_rad) < 0.0000001) angle_ctrl_out =-angle_max_rad;

    //根据公式->{（angle-pwm_min）/（pwm_max+pwm_min）}*(2*angle_max_rad)-angle_max_rad=Ackermann.drive.steering_angle得出，将转向的角度值转换为高电平时间（us）
    angle_out = (0.5-angle_ctrl_out*(1/(2*angle_max_rad)))*(angle_pwm_max_us-angle_pwm_min_us)+angle_pwm_min_us;


    //速度限幅 , 判断浮点数大小应判断差值小于一个很小的数
    if((speed_set - 15) > 0.0000001) speed_set = 15.0;
    if((speed_set + 15) < 0.0000001) speed_set =-15.0;
    //将速度转换为无符号16位整型,便于串口发送
    if(speed_set >= 0.0000001) speed_out = ceil(speed_set * 100) + speed_send_zero;
    if(speed_set <  0.0000001) speed_out = floor(speed_set * 100) + speed_send_zero;
    
    ROS_INFO("angle_ctrl_out = %f  angle_out = %d \n" , angle_ctrl_out , (uint16_t)angle_out);
    ROS_INFO("speed_set = %f  speed_out = %d \n" , speed_set , (uint16_t)speed_out);

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
    send_cmd(uint16_t(speed_send_zero),uint16_t(angle_pwm_mid_us));
    
    ros::init(argc, argv, "art_driver");
    ros::NodeHandle n("~");
    ros::NodeHandle Car_Stop;

    n.param<double>("Angle_P", Angle_P, 1.0);
    n.param<double>("Angle_D", Angle_D, 0.0);
    n.param<std::string>("ackermann_cmd_topic", ackermann_topic, "/cmd_ackermann");

    ros::Subscriber sub_ack_angle = n.subscribe(ackermann_topic, 1, AckermannCallback);

    ros::Subscriber sub_Stop_car = Car_Stop.subscribe("Car_Stop",1,Car_Stop_Callback);
    ros::spin();
    send_cmd(uint16_t(speed_send_zero),uint16_t(angle_pwm_mid_us));
}