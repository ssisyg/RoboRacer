#!/usr/bin/env python3
# Odometry Node

import rospy
import math
import tf
from tf2_ros import TransformBroadcaster 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class OdometryNode:
    def __init__(self):
        rospy.init_node('odometry_node', anonymous=True)
	
        self.telemetry_sub = rospy.Subscriber("/telemetry", AckermannDriveStamped, self.telemetry_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster= TransformBroadcaster()
        
        self.wheelbase = rospy.get_param("~wheelbase", 0.325)

        # Odometry attributes
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = None

    def telemetry_callback(self, msg):
        current_time = msg.header.stamp;

        if self.prev_time is None:
            self.prev_time = current_time
            return

        vel = msg.drive.speed
        angle = msg.drive.steering_angle

        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time

        self.x += vel * math.cos(self.theta) * dt
        self.y += vel * math.sin(self.theta) * dt
        self.theta += (vel / self.wheelbase) * math.tan(angle) * dt

        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
	
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(*quat)

        odom_msg.twist.twist.linear.x = vel
        odom_msg.twist.twist.angular.z = (vel / self.wheelbase) * math.tan(angle)

        self.odom_pub.publish(odom_msg)
        
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*quat)
        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    node = OdometryNode()
    rospy.spin()

