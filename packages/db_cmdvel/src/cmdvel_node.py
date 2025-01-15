#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Header

class CmdVelNode:
    def __init__(self, node_name='db_cmdvel'):
        rospy.init_node(node_name, anonymous=True)
        self.msg_pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)

        self.vel = [0.0, 0.0]
        rospy.Timer(rospy.Duration(0.02), self.timer_cb) # there seems to be a glitch that causes only one wheel to turn

        self.stop_pub = rospy.Publisher('wheels_driver_node/emergency_brake', BoolStamped, queue_size=1)
        rospy.Subscriber('emergency_brake', Bool, self.brake_cb)

        self.set_emerg_brake(False) # disengage emergency brake so we can move

        rospy.loginfo('node started')

    def brake_cb(self, data: BoolStamped):
        self.set_emerg_brake(data.data)

    def set_emerg_brake(self, state):
        rospy.loginfo(f'setting emergency brake to {state}')
        self.stop_pub.publish(BoolStamped(header=Header(stamp=rospy.Time.now()), data=state))

    def timer_cb(self, event):
        msg = Twist2DStamped(header=Header(stamp=rospy.Time.now()), v=self.vel[0], omega=self.vel[1])
        self.msg_pub.publish(msg)
    
    def cmd_vel_cb(self, data: Twist):
        self.vel = [data.linear.x, data.angular.z]
        rospy.loginfo(f'received cmd_vel message: {self.vel}')

if __name__ == '__main__':
    node = CmdVelNode()
    rospy.spin()