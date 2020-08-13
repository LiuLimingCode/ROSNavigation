#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


def odometry_Callback(data):

    global std_msgs_publisher

    std_msgs_data = Float64()
    std_msgs_data.data = int(data.twist.twist.linear.x * 10)

    std_msgs_publisher.publish(std_msgs_data)

if __name__ == "__main__":
    rospy.init_node("odom_speed_to_std_msgs_node")

    odom_topic = rospy.get_param("~odom_topic", "/odom")
    std_msgs_topic = rospy.get_param("~odom_speed_std_msgs_topic", "odom_speed_std_msgs")

    rospy.Subscriber(odom_topic, Odometry, odometry_Callback)
    std_msgs_publisher = rospy.Publisher(std_msgs_topic, Float64, queue_size=1)

    rospy.spin()

