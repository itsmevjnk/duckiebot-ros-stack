#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import WheelEncoderStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, TwistWithCovariance, Twist, Vector3
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

import numpy as np
import tf_conversions

class WheelOdomNode:
    def __init__(self, node_name='wheel_odom'):
        rospy.init_node(node_name, anonymous=True)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)

        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.invert_wheels = rospy.get_param('~invert', False)
        self.imu_reloc = rospy.get_param('~imu_reloc', False) # relocalise with IMU when started

        self.left_pos: int = 0; self.left_msg: WheelEncoderStamped | None = None; rospy.Subscriber('left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_cb)
        self.right_pos: int = 0; self.right_msg: WheelEncoderStamped | None = None; rospy.Subscriber('right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_cb)
        self.t_last: rospy.Time | None = None

        self.enc_resolution: int | None = None

        robot_name = rospy.get_param('~robot_name')
        self.radius = rospy.get_param(f'/{robot_name}/kinematics_node/radius')
        self.baseline = rospy.get_param(f'/{robot_name}/kinematics_node/baseline')

        self.x = 0.0; self.y = 0.0; self.heading = 0.0

        rospy.Subscriber('imu', Imu, self.imu_cb) # for relocalisation
        rospy.Service('relocalise_with_imu', Empty, self.imu_reloc_cb)

        rospy.loginfo('node started')

    def imu_reloc_cb(self, request: EmptyRequest):
        self.imu_reloc = True
        rospy.loginfo('will read message from IMU topic to set orientation')
        return EmptyResponse()
    
    def imu_cb(self, data: Imu):
        if self.imu_reloc:
            if data.orientation.w != 0 and data.orientation.x != 0 and data.orientation.y != 0 and data.orientation.z != 0: # data must be valid
                _, _, self.heading = tf_conversions.transformations.euler_from_quaternion([
                    data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
                ])
                rospy.loginfo(f'received IMU heading {self.heading} rad')
                self.imu_reloc = False

    def left_cb(self, data: WheelEncoderStamped):
        self.left_msg = data
        if self.left_msg is not None and self.right_msg is not None: self.compute_odometry(data.header.stamp)

    def right_cb(self, data: WheelEncoderStamped):
        self.right_msg = data
        if self.left_msg is not None and self.right_msg is not None: self.compute_odometry(data.header.stamp)
        
    def compute_odometry(self, t_now: rospy.Time):
        t_diff: rospy.Duration = abs(self.left_msg.header.stamp - self.right_msg.header.stamp)
        if t_diff > rospy.Duration(0.15):
            discard_left = self.left_msg.header.stamp < self.right_msg.header.stamp
            if discard_left: self.left_msg = None
            else: self.right_msg = None
            rospy.logwarn(f'excessive time skew between left and right wheel encoder messages ({t_diff.to_sec()} s), discarding {"left" if discard_left else "right"}')
            return
        
        left_res = self.left_msg.resolution; right_res = self.right_msg.resolution
        if left_res != right_res:
            rospy.logerr(f'encoder resolution mismatch which is unexpected')
            return
        
        left_pos = self.left_msg.data
        right_pos = -self.right_msg.data
        if self.invert_wheels:
            left_pos = -left_pos; right_pos = -right_pos

        self.left_msg = None; self.right_msg = None

        if self.enc_resolution is not None:            
            d_left = left_pos - self.left_pos
            d_right = right_pos - self.right_pos
            d_left, d_right = (2 * np.pi / self.enc_resolution * np.array([d_left, d_right])).tolist()
            d_array = self.radius / 2 * np.hstack([
                (d_left - d_right) * np.array([np.cos(self.heading), np.sin(self.heading)]),
                np.array([-2 * (d_left + d_right) / self.baseline])
            ])
            vx, vy, omega = (d_array / (t_now - self.t_last).to_sec()).tolist()
            self.x, self.y, self.heading = (np.array([self.x, self.y, self.heading]) + d_array).tolist()
            # rospy.loginfo(f'x={self.x}, y={self.y}, hdg={self.heading}')
            
            hx, hy, hz, hw = tf_conversions.transformations.quaternion_from_euler(0, 0, self.heading)

            msg = Odometry(
                header=Header(stamp=rospy.Time.now(), frame_id=self.odom_frame),
                child_frame_id=self.robot_frame,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self.x, y=self.y, z=0),
                        orientation=Quaternion(x=hx, y=hy, z=hz, w=hw)
                    )
                ),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=vx, y=vy, z=0),
                        angular=Vector3(x=0, y=0, z=omega)
                    )
                )
            )
            self.odom_pub.publish(msg)
        else:
            self.enc_resolution = left_res # initial data
        self.left_pos = left_pos; self.right_pos = right_pos
        self.t_last = t_now

if __name__ == '__main__':
    node = WheelOdomNode()
    rospy.spin()