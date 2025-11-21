#!/usr/bin/env python3
# coding: utf-8

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from ackermann_msgs.msg import AckermannDriveStamped

TWO_PI = 2 * math.pi
DEG_TO_RAD = math.pi / 180.0
MIN_ANGLE = 50
MAX_ANGLE = 90
PERPENDICULAR = 90 * DEG_TO_RAD

class TelemetryNode:
    def __init__(self):
        rospy.init_node('telemetry_node', anonymous=True)

        self.tele_pub = rospy.Publisher("/telemetry", AckermannDriveStamped, queue_size=10)
        self.linear_sub = rospy.Subscriber("/sensors/core", VescStateStamped, self.linear_callback)
        self.servo_sub = rospy.Subscriber("sensors/servo_position_command", Float64, self.servo_callback)

        # current robot specs
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.045)
        self.pole_pairs = rospy.get_param("~pole_pairs", 2.0)
        self.gear_ratio = rospy.get_param("~gear_ratio", 4.5)

        self.linear_speed = 0.0
        self.angle = 0.0  # steering angle in radians

        # Flags to prevent instant publishing
        self.got_linear = False
        self.got_angle = False

    def linear_callback(self, raw_tele_msg):
        raw_electrical_rpm = raw_tele_msg.state.speed / self.pole_pairs
        duty_cycle = raw_tele_msg.state.duty_cycle

        electrical_rpm = raw_electrical_rpm / self.gear_ratio

        raw_linear_speed = electrical_rpm / 60.0 * TWO_PI * self.wheel_radius

        if duty_cycle != 0:
            self.linear_speed = raw_linear_speed / duty_cycle
        else:
            self.linear_speed = 0.0

        self.got_linear = True
        self.publish_telemetry()

    def servo_callback(self, raw_servo_msg):
        deg_angle = (1.0 - raw_servo_msg.data) * 60.0 - 30.0
        self.angle = deg_angle * math.pi / 180.0

        self.got_angle = True
        self.publish_telemetry()

    def publish_telemetry(self):
        if self.got_linear and self.got_angle:
            telemetry_msg = AckermannDriveStamped()
            telemetry_msg.header.stamp = rospy.Time.now()
            telemetry_msg.header.frame_id = "base_link"

            telemetry_msg.drive.steering_angle = self.angle
            telemetry_msg.drive.speed = self.linear_speed

            self.tele_pub.publish(telemetry_msg)


if __name__ == "__main__":
    try:
        node = TelemetryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

