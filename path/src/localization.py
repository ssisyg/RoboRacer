#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

class SlamEKF:
    def __init__(self):
        rospy.init_node('slam_ekf', anonymous=True)

        self.pub = rospy.Publisher('/kalman_slam', PoseStamped, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.rate = rospy.Rate(10.0)

    def run(self):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(
                    "map", "scanmatcher_frame", rospy.Time(0))

                pose_msg = PoseStamped()
                pose_msg.header.stamp = trans.header.stamp
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position = trans.transform.translation
                pose_msg.pose.orientation = trans.transform.rotation

                self.pub.publish(pose_msg)

            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass

            self.rate.sleep()


if __name__ == "__main__":
    pose = SlamEKF()
    pose.run()

