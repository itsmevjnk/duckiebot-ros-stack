#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import TwistStamped

class KinVelNode:
    def __init__(self, node_name='db_kinvel'):
        rospy.init_node(node_name, anonymous=True)

        self.frame_id = rospy.get_param('~frame_id', 'base_link')

        self.pub = rospy.Publisher('kin_vel', TwistStamped, queue_size=1, latch=True)

        # publish empty message first since we assume the robot to be stationary
        empty_msg = TwistStamped(); empty_msg.header.stamp = rospy.Time.now(); empty_msg.header.frame_id = self.frame_id
        self.pub.publish(empty_msg)

        rospy.Subscriber('kinematics_node/velocity', Twist2DStamped, self.msg_cb, queue_size=1)

        rospy.loginfo('node started')

    def msg_cb(self, data: Twist2DStamped):
        msg = TwistStamped()
        msg.header.stamp = data.header.stamp
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = data.v
        msg.twist.angular.z = data.omega
        self.pub.publish(msg)

if __name__ == '__main__':
    node = KinVelNode()
    rospy.spin()