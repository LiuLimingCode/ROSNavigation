#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, Vector3
from ackermann_msgs.msg import AckermannDriveStamped

def Twist_Cmd_Callback(data):

    global ack_publisher
    global angular_last

    ack_cmd = AckermannDriveStamped()
    ack_cmd.header.stamp = rospy.Time.now()

    ack_cmd.drive.speed = data.linear.x
    ack_cmd.drive.steering_angle = data.angular.z

    ack_publisher.publish(ack_cmd)
    angular_last = data.angular.z

if __name__ == "__main__":
    rospy.init_node("cmd_to_ackermann_node")

    global ack_publisher
    global angular_last
    angular_last = 0.0

    twist_cmd_topic = rospy.get_param("~twist_cmd_topic", "/cmd_vel")
    ackermann_cmd_topic = rospy.get_param("~ackermann_cmd_topic", "/cmd_ackermann")

    ack_publisher = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    rospy.Subscriber(twist_cmd_topic, Twist, Twist_Cmd_Callback)

    rospy.spin()

