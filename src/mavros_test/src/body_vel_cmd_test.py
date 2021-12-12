#!/usr/bin/env python

import rospy
import mavros
import numpy as np

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
        self.v_max_h = 1
        # maximum vertical velocity
        self.v_max_v = 1

        self.vel_setpoint_x = 0
        self.vel_setpoint_y = 0
        self.vel_setpoint_z = 0

        self.state = State()
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_cb)

        self.pid_vel_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    def state_cb(self, msg):
        self.state = msg


if __name__ == '__main__':
    rospy.init_node('body_vel_cmd_test', anonymous=True)
    cmd = commander()
    rate = rospy.Rate(20)
    position_home = PositionTarget()
    position_home.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    position_home.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE
    position_home.position.x = 0
    position_home.position.y = 0
    position_home.position.z = 2.5
    position_home.velocity.x = 0
    position_home.velocity.y = 0
    position_home.velocity.z = 0
    position_home.acceleration_or_force.x = 0
    position_home.acceleration_or_force.y = 0
    position_home.acceleration_or_force.z = 0
    position_home.yaw = np.pi / 2
    position_home.yaw_rate = 0
    i = 0
    while not rospy.is_shutdown():
        if cmd.state.mode != "OFFBOARD" and (not cmd.state.armed):
            print('The drone is not armed or not in offboard mode')
            continue

        if i < 200:
            cmd.set_point.header.stamp = rospy.Time.now()
            cmd.set_point.velocity.x = 1
            cmd.set_point.velocity.y = 0
            cmd.set_point.velocity.z = 0
            cmd.set_point.yaw = 0
            cmd.pid_vel_pub.publish(cmd.set_point)
            # print('send a body velocity command (1, 0, 0)')
            i += 1

        elif (200 <= i < 500):
            position_home.header.stamp = rospy.Time.now()
            cmd.pid_vel_pub.publish(position_home)
            #print('we are sending command to go back')
            i += 1
        else:
            i = 0

        rate.sleep()

        

       
