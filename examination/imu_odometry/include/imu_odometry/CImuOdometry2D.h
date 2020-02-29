/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
******************************************************************************************** */

#ifndef CImuOdometry2D_H
#define CImuOdometry2D_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


class CImuOdometry2D
{
public:
    CImuOdometry2D();
    ~CImuOdometry2D();
    bool is_initialized();
    bool scan_available();
    void Init();
    void odometryCalculation();

    std::string         imu_topic;
    std::string         odom_topic;
    std::string         base_frame;
    std::string         imu_frame;
    std::string         odom_frame;
    std::string         init_pose_from_topic;
    bool                publish_tf;
    double              freq;

protected:
    
    sensor_msgs::LaserScan      last_data;
    bool                        is_first_data, is_data_available, GT_pose_initialized;
    tf::TransformListener       tf_listener;          //Do not put inside the callback
    tf::TransformBroadcaster    odom_broadcaster;
    nav_msgs::Odometry          initial_robot_pose;

    //Subscriptions & Publishers
    ros::Subscriber imu_sub, initPose_sub;
    ros::Publisher odom_pub;

    //CallBacks
    void ImuCallBack(const sensor_msgs::Imu::ConstPtr& data);
    void initPoseCallBack(const nav_msgs::Odometry::ConstPtr& data);



	mrpt::poses::CPose3D laser_pose;
	mrpt::poses::CPose3D laser_oldpose;
    mrpt::poses::CPose3D robot_pose;
    mrpt::poses::CPose3D robot_oldpose;
	bool test;
    std::vector<double> last_m_lin_speeds;
    std::vector<double> last_m_ang_speeds;


	// Methods
    isDataAvailable(void) const;
};

#endif
