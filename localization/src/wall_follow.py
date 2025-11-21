#!/usr/bin/env python3
# coding: utf-8

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

DEG_TO_RAD = math.pi / 180.0
MIN_ANGLE = 50
MAX_ANGLE = 90
PERPENDICULAR = 90 * DEG_TO_RAD

class WallFollowNode:
    def __init__(self):
        rospy.init_node('wall_follow_node', anonymous=True)

        # Wall-following parameters
        self.L = 1.0  # lookahead distance
        
        # min distance from wall desired (narrowest part of track). 
        if not rospy.has_param("~distance_setpoint"):
            rospy.logwarn("Distance setpoint not given. Using default (1.0)")
            self.distance_setpoint = 1.0
        else:
            self.distance_setpoint = rospy.get_param("~distance_setpoint")

        self.out_of_range = self.distance_setpoint * 5

        # Angles for the two lidar beams (RIGHT WALL)
        self.a_angle = -1 * MIN_ANGLE * DEG_TO_RAD
        self.b_angle = -1 * MAX_ANGLE * DEG_TO_RAD
        self.theta = 40 * DEG_TO_RAD
    
        self.down_sample_factor = rospy.get_param("~down_sample", 1)

        # PID coefficients
        if not rospy.has_param("~kp"):
            rospy.logwarn("PIDs in launch file incomplete. Using defaults.")

        self.kp = rospy.get_param("~kp", 100.0)
        self.ki = rospy.get_param("~ki", 1.0)
        self.kd = rospy.get_param("~kd", 5.0)
        
        # PID state
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = None

        # Velocity limits
        self.straight_vel = 0.6
        self.low_turn_vel = 0.40
        self.med_turn_vel = 0.30
        self.high_turn_vel = 0.25

        self.drive_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def get_range(self, scan_msg, angle):
        """
        Return the range measurement at a given angle.
        """
        # clip angle to sensor bounds
        if angle < scan_msg.angle_min:
            angle = scan_msg.angle_min
        elif angle > scan_msg.angle_max:
            angle = scan_msg.angle_max

        idx = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
        idx = max(0, min(idx, len(scan_msg.ranges)-1))

        distance = scan_msg.ranges[idx]
        if math.isinf(distance) or math.isnan(distance):
            return scan_msg.range_max
        else:
            return distance
    
    # get aggregate error (of one side of the scans, either left or right)
    def get_agg_error(self, scan_msg, min_angle, max_angle, down_sample_factor):
        angle = min_angle
        D_t = 0.0
        D_t_plus_lookahead = 0.0

        num = abs(int((max_angle - min_angle) / scan_msg.angle_increment))
        num /= down_sample_factor
        
        angle_increment = down_sample_factor * scan_msg.angle_increment

        ortho_scan = self.get_range(scan_msg, PERPENDICULAR)
        
        while angle <= max_angle:
            tmp = self.get_range(scan_msg, angle)
            if tmp > self.out_of_range:
                #angle += scan_msg.angle_increment
                angle += angle_increment
                num -= 1
                continue
            
            alpha = math.atan2(tmp * math.cos(PERPENDICULAR - abs(angle)), tmp * math.sin(PERPENDICULAR - abs(angle)))
            
            tmp_d_t = tmp * math.cos(alpha)            
            tmp_d_t_plus_lookahead = tmp_d_t + self.L * math.sin(alpha)
          
            D_t += tmp_d_t
            D_t_plus_lookahead += tmp_d_t_plus_lookahead
            
            #angle += scan_msg.angle_increment
            angle += angle_increment
            
        avg_D_t = D_t / num
        avg_D_t_plus_lookahead = D_t_plus_lookahead / num
        error = self.distance_setpoint - avg_D_t_plus_lookahead
        
        return error, avg_D_t, avg_D_t_plus_lookahead, num

    	    	
    
    # def get_error(self, scan_msg):
    #     a = self.get_range(scan_msg, self.a_angle)
    #     b = self.get_range(scan_msg, self.b_angle)
    #
    #     alpha = math.atan2(a * math.cos(self.theta) - b,
    #                        a * math.sin(self.theta))
    # 
    #     D_t = b * math.cos(alpha)
    #     D_t_plus_1 = D_t + self.L * math.sin(alpha)
    # 
    #     error = self.distance_setpoint - D_t_plus_1
    #     return error

    def pid_control(self, error, current_time):
        if self.prev_time is None:
            delta_t = 0.0
        else:
            delta_t = current_time - self.prev_time

        derivative_error = (error - self.prev_error) / delta_t if delta_t > 0 else 0.0
        self.integral_error += error * delta_t

        angle = (self.kp * error +
                 self.ki * self.integral_error +
                 self.kd * derivative_error)

        # limit angle to ±30 deg
        angle = max(min(angle, 30), -30)
        self.prev_error = error
        self.prev_time = current_time
        
        
        return angle

    def velocity_limiter(self, angle):
        abs_angle = abs(angle)
        
        if 0 < abs_angle and abs_angle < 5.0:
            return self.straight_vel
        elif 5.0 <= abs_angle  and abs_angle < 13.0:
            return self.low_turn_vel
        elif 13.0  <= abs_angle and abs_angle < 20.0:
            return self.med_turn_vel
        else:    
            return self.high_turn_vel

    def scan_callback(self, scan_msg):
        start_exec_time = rospy.Time.now()
        # error = self.get_error(scan_msg)
        left_error, _ , _, _ = self.get_agg_error(scan_msg, MIN_ANGLE * DEG_TO_RAD, MAX_ANGLE * DEG_TO_RAD, self.down_sample_factor)
        # right side's angles become inverted 
        right_error, _, _, _ = self.get_agg_error(scan_msg, -1 * MAX_ANGLE * DEG_TO_RAD, -1 * MIN_ANGLE * DEG_TO_RAD, self.down_sample_factor)
        # rospy.loginfo("LEFT_ERR: % .3f   RIGHT_ERR: % .3f", left_error, right_error)

        # SUS.
        ## error = left_error - right_error
        error = right_error - left_error
        # rospy.loginfo("COMBINED ERR: % .3f", error)
        
        current_time = scan_msg.header.stamp.to_sec()

        angle = self.pid_control(error, current_time)
        velocity = self.velocity_limiter(angle)

        steer_angle = (angle * -1 + 30.0) / 60.0
        drive_msg = Twist()
        drive_msg.angular.z = steer_angle
        drive_msg.linear.x = velocity

        # rospy.loginfo_throttle(1.0, f"Velocity: {velocity:.2f} m/s, Steering Angle: {math.degrees(angle):.2f} deg")
 
        self.drive_pub.publish(drive_msg)
        
        end_exec_time = rospy.Time.now()
        
        rospy.loginfo("EXEC_TIME: %.5f", (end_exec_time - start_exec_time).to_sec())


if __name__ == "__main__":
    try:
        node = WallFollowNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
