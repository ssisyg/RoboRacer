#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import csv

poses = []

def callback(msg):
    new_point = (msg.pose.position.x, msg.pose.position.y)
    
    # Only save if moved at least 2 cm from last point
    if len(poses) == 0 or distance(poses[-1], new_point) > 0.02:
        poses.append(new_point)

def distance(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

def save_csv():
    if len(poses) == 0:
        rospy.logwarn("No poses saved!")
        return

    with open("waypoints.csv", "w", newline='') as f:
        writer = csv.writer(f)
        for p in poses:
            writer.writerow([p[0], p[1]])

    rospy.loginfo(f"Saved {len(poses)} poses to waypoints.csv")

if __name__ == '__main__':
    rospy.init_node("save_trajectory")

    # Subscribe to your PoseStamped topic (adjust the topic name if needed)
    rospy.Subscriber("/kalman_slam", PoseStamped, callback)

    # Register shutdown hook to save the CSV when the node exits
    rospy.on_shutdown(save_csv)

    rospy.loginfo("save_trajectory node started. Listening for poses...")
    rospy.spin()

