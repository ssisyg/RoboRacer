#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math

def publish_imu():
    sensor = mpu6050(0x68)
    pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
    rospy.init_node('imu_node', anonymous=True)
    rate = rospy.Rate(50)

    imu_msg = Imu()
    imu_msg.header.frame_id = "imu_link"
    
    gyro_bias_x = rospy.get_param("~gyro_bias/x", 0.0)
    gyro_bias_y = rospy.get_param("~gyro_bias/y", 0.0)
    gyro_bias_z = rospy.get_param("~gyro_bias/z", 0.0)

    
    rospy.loginfo("Using gyro bias: x=%.5f, y=%.5f, z=%.5f",
              gyro_bias_x, gyro_bias_y, gyro_bias_z)
    while not rospy.is_shutdown():
        accel = sensor.get_accel_data()
        raw_gyro = sensor.get_gyro_data()
        gyro = {'x': raw_gyro['x'] + gyro_bias_x,
                'y': raw_gyro['y'] + gyro_bias_y,
                'z': raw_gyro['z'] + gyro_bias_z
                }
        
        # imu orientation is scuffed, so fix it in your config files!!
        # these are not modified !!
        ax = accel['x']
        ay = accel['y']
        az = accel['z']

        gx = gyro['x']
        gy = gyro['y']
        gz = gyro['z']

        imu_msg.header.stamp = rospy.Time.now()

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)
        
        imu_msg.linear_acceleration_covariance = [0.05, 0, 0,
                                          0, 0.05, 0,
                                          0, 0, 0.05]

        imu_msg.angular_velocity_covariance = [0.01, 0, 0,
                                                0, 0.01, 0,
                                                0, 0, 0.01]
    
        imu_msg.orientation_covariance = [-1, 0, 0,
                                            0, 0, 0,
                                            0, 0, 0]  # -1 means orientation not provided

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_imu()
    except rospy.ROSInterruptException:
        pass

