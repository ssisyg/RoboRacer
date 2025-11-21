#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class CmdVelToVesc:
    def __init__(self):
        # Parameters
        self.center_angle = 0.5
        self.steering_angle = self.center_angle

        # Servo Specs
        self.max_angle = 30.0
        self.max_angle_rad = math.radians(self.max_angle)

        rospy.init_node('cmdvel_to_vesc')

        self.motor_pub = rospy.Publisher(
            '/commands/motor/duty_cycle', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher(
            '/commands/servo/position', Float64, queue_size=1)

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def cmd_vel_callback(self, msg):
        duty = self.clamp(msg.linear.x * 0.25, -0.2, 0.2)
        
        # Incremental steering control
        #step = 0.05
        #if msg.angular.z > 0.1:
        #    self.steering_angle += step
        #elif msg.angular.z < -0.1:
        #    self.steering_angle -= step
        #else:
        #    pass  # optionally recenter
        
        #if abs(msg.angular.z - 99.0) < 1e-2:
        #    self.steering_angle = 0.5

        self.steering_angle = msg.angular.z
        # servo motor seems to be mounted opposite. 
        # 0.5 ~ 1 -> 0 ~ 0.5
        # 0 ~ 0.5 -> 0.5 ~ 1
        # self.steering_angle = (self.steering_angle - 0.5) * -1.0 + 0.5
        self.steering_angle = self.clamp(self.steering_angle, 0.0, 1.0)

        rospy.loginfo("Duty=%.2f, Steering=%.2f", duty, self.steering_angle)

        self.motor_pub.publish(Float64(data=duty))
        self.servo_pub.publish(Float64(data=self.steering_angle))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CmdVelToVesc()
        node.spin()
    except KeyboardInterrupt:
        pass
