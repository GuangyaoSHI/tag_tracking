#!/usr/bin/env python

import rospy
import mavros

from geometry_msgs.msg import PoseStamped


pose_body = PoseStamped()
def tag_pose_cb(tag_pose):
    pose_body.pose.position.x = tag_pose.pose.position.z + 0.1
    pose_body.pose.position.y = -tag_pose.pose.position.x
    pose_body.pose.position.z = -tag_pose.pose.position.y
    pose_body.pose.orientation = tag_pose.pose.orientation
    

tag_pose_sub = rospy.Subscriber('/tag_detections/tagpose', PoseStamped, tag_pose_cb)
pose_body_pub = rospy.Publisher('/tag_pose_body', PoseStamped, queue_size=10)

rospy.init_node('camera2body_test', anonymous=True)


while not rospy.is_shutdown():
    pose_body_pub.publish(pose_body)
