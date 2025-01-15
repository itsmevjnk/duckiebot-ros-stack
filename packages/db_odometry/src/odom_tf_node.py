#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFNode:
    def __init__(self, node_name='wheel_odom'):
        rospy.init_node(node_name, anonymous=True)

        self.tf_broadcaster = TransformBroadcaster()
        rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1)

        rospy.loginfo('node started')

    def odom_cb(self, data: Odometry):
        t = TransformStamped()

        t.header.stamp = data.header.stamp
        t.header.frame_id = data.header.frame_id
        t.child_frame_id = data.child_frame_id

        t.transform.translation.x = data.pose.pose.position.x
        t.transform.translation.y = data.pose.pose.position.y
        t.transform.translation.z = data.pose.pose.position.z
        t.transform.rotation = data.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    node = OdomTFNode()
    rospy.spin()