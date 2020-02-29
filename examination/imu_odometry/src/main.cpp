#include <ros/ros.h>
#include "imu_odometry/CImuOdometry2D.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ImuOdometry");

    CImuOdometry2D myImuOdom;

    ros::Rate loop_rate(myImuOdom.freq);
    while (ros::ok())
    {
        ros::spinOnce();        //Check for new laser scans
        ROS_INFO("LOOP");
        if( myImuOdom.is_initialized() && myImuOdom.scan_available() )
        {            
            //Process odometry estimation
            myImuOdom.odometryCalculation();
            ROS_INFO("odom calc");
        }
        else
        {
            ROS_WARN("Waiting for imu_data....") ;
        }

        loop_rate.sleep();
    }
    return(0);
}
