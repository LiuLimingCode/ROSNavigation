#include "imu_odometry/CImuOdometry2D.h"

using namespace std;

CImuOdometry2D::CImuOdometry2D()
{	
    ROS_INFO("Initializing IMU_Odometry node...");

    ros::NodeHandle n;
    n.param<std::string>("imu_topic", imu_topic, "/imu");
    n.param<std::string>("odom_topic", odom_topic, "/odom_imu");
    n.param<std::string>("base_frame", base_frame, "/base_link");
    n.param<std::string>("odom_frame", odom_frame, "/odom");
    n.param<std::string>("imu_frame", imu_frame, "/imu_frame");
    n.param<bool>("publish_tf", publish_tf, true);
    n.param<double>("freq", freq, 10.0);

    odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 5);
    imu_sub = n.subscribe<sensor_msgs::LaserScan>(imu_topic, 1, &CImuOdometry2D::ImuCallBack, this);

    is_first_data = true;
}

CImuOdometry2D::~CImuOdometry2D()
{
}


bool CLaserOdometry2D::is_data_available(void) const
{
    return is_data_available;
}

void CLaserOdometry2D::odometryCalculation()
{
    PoseUpdate();
    is_data_available = false;     //avoids the possibility to run twice on the same laser scan
}

void CLaserOdometry2D::Reset(CPose3D ini_pose, CObservation2DRangeScan scan)
{
    //Set the initial pose
    laser_pose = ini_pose;
    laser_oldpose = ini_pose;
}


void CLaserOdometry2D::PoseUpdate()
{
	
  //first, we'll publish the odometry over tf
  //---------------------------------------
  if (publish_tf)
  {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = robot_pose.x();
    odom_trans.transform.translation.y = robot_pose.y();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(robot_pose.yaw());
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
  }

    //next, we'll publish the odometry message over ROS
    //-------------------------------------------------
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;
    //set the position
    odom.pose.pose.position.x = robot_pose.x();
    odom.pose.pose.position.y = robot_pose.y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_pose.yaw());
    //set the velocity
    odom.child_frame_id = base_frame_id;
    #ifdef rplidar 
    	odom.twist.twist.linear.x = lin_speed;    //linear speed
    #else
    	odom.twist.twist.linear.x = -lin_speed;    //linear speed
    #endif
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = ang_speed;   //angular speed

	
    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[14] = 0.1;
    odom.pose.covariance[21] = 0.0025;
    odom.pose.covariance[28] = 0.0025;
    odom.pose.covariance[25] = 0.0025;

    odom.twist.covariance[0] = 0.25;
    odom.twist.covariance[7] = 0.25;
    odom.twist.covariance[14] = 0.1;
    odom.twist.covariance[21] = 0.02;
    odom.twist.covariance[28] = 0.02;
    odom.twist.covariance[25] = 0.02;

    odom_pub.publish(odom);
}



//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2D::initPoseCallBack(const nav_msgs::Odometry::ConstPtr& data)
{

}

void CLaserOdometry2D::LaserCallBack(const sensor_msgs::Imu::ConstPtr& data)
{
    last_data = *data;

    is_data_available = true;

}


