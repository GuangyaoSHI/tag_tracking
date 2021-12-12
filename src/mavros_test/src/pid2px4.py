#!/usr/bin/env python

import rospy
import mavros

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import Float64
from math import sqrt


class commander:
    def __init__(self):
        self.set_point = PositionTarget()
        self.set_point.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.set_point.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                                            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                            PositionTarget.IGNORE_YAW_RATE
        # maximum horizontal velocity
        self.v_max_h = 0.3
        # maximum vertical velocity
        self.v_max_v = 0.2

        self.vel_setpoint_x = 0
        self.vel_setpoint_y = 0
        self.vel_setpoint_z = 0

        self.state = State()
        #self.state_sub = rospy.Subscriber(mavros.get_topic('state'), State, self.state_cb)
        self.pid_vx_sub = rospy.Subscriber('/x/control_effort', Float64, self.vx_cmd_cb)
        self.pid_vy_sub = rospy.Subscriber('/y/control_effort', Float64, self.vy_cmd_cb)
        self.pid_vz_sub = rospy.Subscriber('/z/control_effort', Float64, self.vz_cmd_cb)

        self.pid_vel_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    def vx_cmd_cb(self, msg):
        self.vel_setpoint_x = msg.data

    def vy_cmd_cb(self, msg):
        self.vel_setpoint_y = msg.data

    def vz_cmd_cb(self, msg):
        self.vel_setpoint_z = msg.data

    def state_cb(self, msg):
        self.state = msg


if __name__ == '__main__':
    rospy.init_node('pid_merge_node', anonymous=True)
    cmd = commander()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        #if cmd.state.mode != "OFFBOARD" and (not cmd.state.armed):
        #    print('The drone is not armed or not in offboard mode')
        #    continue

        cmd.set_point.header.stamp = rospy.Time.now()
        v = sqrt(pow(cmd.vel_setpoint_x, 2) + pow(cmd.vel_setpoint_y, 2))
        if v > cmd.v_max_h:
            scale = cmd.v_max_h / v
        else:
            scale = 1
        cmd.set_point.velocity.x = cmd.vel_setpoint_x * scale
        cmd.set_point.velocity.y = cmd.vel_setpoint_y * scale
        cmd.set_point.velocity.z = min(cmd.vel_setpoint_z, cmd.v_max_v)
        cmd.set_point.yaw = 0
        cmd.pid_vel_pub.publish(cmd.set_point)
        rate.sleep()
