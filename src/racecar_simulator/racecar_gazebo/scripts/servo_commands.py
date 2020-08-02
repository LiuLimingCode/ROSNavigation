#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

class ServoCommandNode:

    def __init__(self):

        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)
        ackermann_output_topic = rospy.get_param("~ackermann_output_topic", "ackermann_cmd_mux/output")
        left_rear_wheel_command_topic = rospy.get_param("~left_rear_wheel_command_topic", "left_rear_wheel_velocity_controller/command")
        right_rear_wheel_command_topic = rospy.get_param("~right_rear_wheel_command_topic", "right_rear_wheel_velocity_controller/command")
        left_front_wheel_command_topic = rospy.get_param("~left_front_wheel_command_topic", "left_front_wheel_velocity_controller/command")
        right_front_wheel_command_topic = rospy.get_param("~right_front_wheel_command_topic", "right_front_wheel_velocity_controller/command")
        left_steering_command_topic = rospy.get_param("~left_steering_command_topic", "left_steering_hinge_position_controller/command")
        right_steering_command_topic = rospy.get_param("~right_steering_command_topic", "right_steering_hinge_position_controller/command")

        rospy.Subscriber(ackermann_output_topic, AckermannDriveStamped, self.set_throttle_steer)

        self.pub_vel_left_rear_wheel = rospy.Publisher(left_rear_wheel_command_topic, Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher(right_rear_wheel_command_topic, Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher(left_front_wheel_command_topic, Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher(right_front_wheel_command_topic, Float64, queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher(left_steering_command_topic, Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher(right_steering_command_topic, Float64, queue_size=1)

    def set_throttle_steer(self, data):

        # Velocity is in terms of radians per second.
        # Want to go 1 m/s with a wheel of radius 0.05m. This translates to 19.97 radians per second, roughly 20.
        # However, at a multiplication factor of 20 speed is half of what it should be, so doubled to 40.
        throttle = data.drive.speed * 0.05 * 19.97 / self.wheel_radius
        steer = data.drive.steering_angle

        self.pub_vel_left_rear_wheel.publish(throttle - 0.45 * math.tan(steer) * throttle)
        self.pub_vel_right_rear_wheel.publish(throttle + 0.45 * math.tan(steer) * throttle)
        self.pub_vel_left_front_wheel.publish(throttle)
        self.pub_vel_right_front_wheel.publish(throttle)
        self.pub_pos_left_steering_hinge.publish(steer)
        self.pub_pos_right_steering_hinge.publish(steer)

if __name__ == '__main__':
    rospy.init_node('servo_commands', anonymous=True)
    node = ServoCommandNode()
    rospy.spin()
