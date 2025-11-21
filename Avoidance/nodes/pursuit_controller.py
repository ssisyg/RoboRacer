#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist

class PurePursuitController:
    def __init__(self):
        rospy.init_node('pure_pursuit_controller')

        # --- Parameters (CRITICAL TO TUNE!) ---
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 1.5) # m
        self.max_speed = rospy.get_param('~max_speed', 3.0) # m/s (corresponds to linear.x in Twist)
        self.min_speed = rospy.get_param('~min_speed', 1.0) # m/s
        self.wheelbase = rospy.get_param('~wheelbase', 0.325) # m, distance between front and rear axles
        self.k_p_speed = rospy.get_param('~k_p_speed', 0.5) # Proportional gain for speed control based on steering angle
        
        # --- Internal Variables ---
        self.target_path = None
        self.current_pose = None
        
        # --- ROS Subscribers/Publishers ---
        self.path_sub = rospy.Subscriber('/target_path', Path, self.path_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseStamped, self.pose_callback, queue_size=1) # Or your localization topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Main control loop runs at 50 Hz
        rospy.Timer(rospy.Duration(0.02), self.control_loop)

    def path_callback(self, msg):
        self.target_path = msg

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        
    def control_loop(self, event):
        if self.target_path is None or self.current_pose is None or not self.target_path.poses:
            self.publish_stop_command()
            return
            
        # Find the lookahead point
        lookahead_point, target_vel = self.get_lookahead_point()
        
        if lookahead_point is None:
            self.publish_stop_command()
            return

        # Calculate steering angle (alpha)
        q = self.current_pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        alpha = np.arctan2(lookahead_point.y - self.current_pose.position.y, 
                           lookahead_point.x - self.current_pose.position.x) - yaw
        
        # Pure pursuit formula
        # The steering_angle value might need to be mapped to your servo's range (0.0 to 1.0)
        # Here we assume angular.z is directly used or mapped by the CmdVelToVesc node
        steering_angle = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), self.lookahead_distance)
        
        # Simple speed control: slow down on sharp turns
        speed = self.max_speed - abs(steering_angle) * self.k_p_speed
        speed = np.clip(speed, self.min_speed, self.max_speed)

        # Publish command
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering_angle # This is in radians. Your CmdVelToVesc maps this.
        self.cmd_vel_pub.publish(cmd)

    def get_lookahead_point(self):
        path_points = self.target_path.poses
        my_pos = self.current_pose.position
        
        # Find the closest point on the path to the car's current position
        dists = [np.hypot(p.pose.position.x - my_pos.x, p.pose.position.y - my_pos.y) for p in path_points]
        closest_idx = np.argmin(dists)

        # Search from the closest point to find the lookahead point
        lookahead_idx = closest_idx
        for i in range(closest_idx, len(path_points)):
            dist_to_point = np.hypot(path_points[i].pose.position.x - my_pos.x, 
                                     path_points[i].pose.position.y - my_pos.y)
            if dist_to_point >= self.lookahead_distance:
                lookahead_idx = i
                break
        
        # If no point is far enough, use the last point
        if lookahead_idx == closest_idx:
            lookahead_idx = len(path_points) - 1

        return path_points[lookahead_idx].pose.position, self.max_speed

    def publish_stop_command(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0 # Or your neutral steering value
        self.cmd_vel_pub.publish(cmd)

if __name__ == '__main__':
    try:
        node = PurePursuitController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass