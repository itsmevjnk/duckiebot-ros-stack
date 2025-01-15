#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

import numpy as np
from scipy.spatial.transform import Rotation

class ImuNode:
    def __init__(self, node_name='imu'):
        rospy.init_node(node_name, anonymous=True)
        self.pub = rospy.Publisher('imu', Imu, queue_size=1)
        rospy.Subscriber('imu_node/data', Imu, self.imu_cb, queue_size=1)

        self.imu_frame = rospy.get_param('~frame_id', 'imu_link')
        self.calib_pts = rospy.get_param('~calib_pts', 0) # number of data points to be collected for offset calibration (0 = disable calibration)

        self.offset: np.ndarray | None = None # IMU offset
        self.linacc_cov: list[float] = [0.0] * 9 # linear acceleration covariance
        self.angvel_cov: list[float] = [0.0] * 9 # angular velocity covariance
        self.offset_points = [] # data points used to compute offset

        self.brake_pub = rospy.Publisher('emergency_brake', Bool, queue_size=1)

        if self.calib_pts > 0:
            rospy.loginfo('IMU calibration in progress - do not move robot!')
            self.brake_pub.publish(Bool(True))
        else:
            rospy.loginfo('IMU calibration disabled - will only rename frame')

    def imu_cb(self, data: Imu):
        if self.calib_pts > 0:
            if self.offset is None: # calibration phase
                self.offset_points.append([
                    data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
                    data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z
                ])
                if len(self.offset_points) == self.calib_pts: # enough data points have been gathered, time to calibrate
                    self.brake_pub.publish(Bool(False))
                    points = np.array(self.offset_points)
                    self.offset = np.mean(points, axis=0)
                    points -= self.offset; self.offset = self.offset.tolist()
                    rospy.loginfo(f'calibrated offset: {self.offset}')

                    # calculate covariance
                    self.linacc_cov = np.cov(points[:,:3], rowvar=False).flatten().tolist()
                    rospy.loginfo(f'linear acceleration covariance: {self.linacc_cov}')
                    self.angvel_cov = np.cov(points[:,3:], rowvar=False).flatten().tolist()
                    rospy.loginfo(f'angular velocity covariance: {self.angvel_cov}')

                else: return # do not return IMU data until we're ready
            
            # subtract offset
            data.linear_acceleration.x -= self.offset[0]
            data.linear_acceleration.y -= self.offset[1]
            data.linear_acceleration.z -= self.offset[2]
            data.angular_velocity.x -= self.offset[3]
            data.angular_velocity.y -= self.offset[4]
            data.angular_velocity.z -= self.offset[5]

            data.linear_acceleration_covariance = self.linacc_cov
            data.angular_velocity_covariance = self.angvel_cov

        data.header.frame_id = self.imu_frame
        self.pub.publish(data)

if __name__ == '__main__':
    node = ImuNode()
    rospy.spin()