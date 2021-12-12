#!/usr/bin/env python

import rospy
import mavros

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import Float64
from math import sqrt


class commander:
    def __init__(self):
        self.pos_setpoint_x = Float64()
        self.pos_setpoint_x.data = 0.2
        self.pos_setpoint_y = Float64()
        self.pos_setpoint_y.data = 0
        self.pos_setpoint_z = Float64()
        self.pos_setpoint_z.data = -1.2

        self.state = State()
        #self.state_sub = rospy.Subscriber(mavros.get_topic('state'), State, self.state_cb)
        self.pid_rel_x_pub = rospy.Publisher('/ref_body_x', Float64, queue_size=10)
        self.pid_rel_y_pub = rospy.Publisher('/ref_body_y', Float64, queue_size=10)
        self.pid_rel_z_pub = rospy.Publisher('/ref_body_z', Float64, queue_size=10)

    #def state_cb(self, msg):
    #    self.state = msg


if __name__ == '__main__':
    rospy.init_node('reference_body_node', anonymous=True)
    cmd = commander()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
#        if cmd.state.mode != "OFFBOARD" and (not cmd.state.armed):
#            print('The drone is not armed or not in offboard mode')
#	    print('cannot send reference point wrt body')
#            continue

        cmd.pid_rel_x_pub.publish(cmd.pos_setpoint_x)
        cmd.pid_rel_y_pub.publish(cmd.pos_setpoint_y)
        cmd.pid_rel_z_pub.publish(cmd.pos_setpoint_z)
        rate.sleep()
