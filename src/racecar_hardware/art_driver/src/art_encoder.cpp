#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>


std::string odom_topic;
std::string frame_id;
std::string child_frame_id;
int publish_frequency;

serial::Serial ser;

std::string serial_port;

int count = 0;
double speed = 0;
ros::Publisher pub;

void TimerCallBack(const ros::TimerEvent&)
{
    unsigned char data_size;
    if(data_size = ser.available())
    {
        unsigned char tmpdata[data_size];

        nav_msgs::Odometry odom;

        ser.read(tmpdata,data_size);

        odom.header.stamp = ros::Time::now();
        odom.header.seq = count;
        odom.header.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;
        odom.pose.covariance[0] = 0.00001;
        odom.pose.covariance[7] = 0.00001;
        odom.pose.covariance[14] = 1000000000000.0;
        odom.pose.covariance[21] = 1000000000000.0;
        odom.pose.covariance[28] = 1000000000000.0;
        odom.pose.covariance[35] = 0.001;

        if((tmpdata[0] == 0x00) && (tmpdata[1] == 0xaa))
        {
            uint16_t check = 0,check_receive = 0;
            check_receive = tmpdata[4];
            check = (tmpdata[2] + tmpdata[3]) & 0xFF;
            if((check == check_receive) && (tmpdata[5] == 0xBB))
            {
                speed =(((tmpdata[3] << 8) | tmpdata[2]) - 32767) / 100.0; 
                odom.twist.twist.linear.x = speed;
                pub.publish(odom);
                //  ROS_INFO("ASDJFAJKSFD %f",speed);
                count++;
            }
        }
        ros::spinOnce();
    }
}

int main(int argc,char **argv)
{
    
    ros::init(argc,argv,"encoder_speed");
    ros::NodeHandle n;
    n.param<std::string>("odom_pub_topic", odom_topic, "/odom_rf2o");
    n.param<std::string>("serial_port", serial_port, "/dev/car");
    n.param<std::string>("odom_frame_id", frame_id, "odom");
    n.param<std::string>("base_frame_id", child_frame_id, "base_footprint");
    n.param<int>("publish_frequency", publish_frequency, 50);
    pub = n.advertise<nav_msgs::Odometry>(odom_topic,500);

    try
    {
        ser.setPort(serial_port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("Unable to open port");
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Timer timer = n.createTimer(ros::Duration((1.0) / publish_frequency), &TimerCallBack); // Duration(0.05) -> 20Hz

    ros::spin();
}