#!/usr/bin/env python3
import rospy
from mpu6050 import mpu6050
import yaml
import os

def calibrate_and_save(path):
    sensor = mpu6050(0x68)
    samples = 2000
    sum_x = sum_y = sum_z = 0.0

    print("Calibrating gyro — keep IMU completely still")

    for _ in range(samples):
        g = sensor.get_gyro_data()
        sum_x += g['x']
        sum_y += g['y']
        sum_z += g['z']
        rospy.sleep(0.01)

    bias = {
        "gyro_bias": {
            "x": round(sum_x / samples, 5),
            "y": round(sum_y / samples, 5),
            "z": round(sum_z / samples, 5)
        }
    }

    with open(path, "w") as f:
        yaml.dump(bias, f, default_flow_style=False)

    print("Calibration complete. Saved to:", path)
    print(bias)

if __name__ == '__main__':
    rospy.init_node("imu_calibration_node", anonymous=True)
    pkg_path = os.path.dirname(os.path.abspath(__file__ + "/.."))
    yaml_path = os.path.join(pkg_path, "config", "imu_calibration.yaml")
    calibrate_and_save(yaml_path)
