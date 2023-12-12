#!/usr/bin/env python3

import rospy
import tf
import math as m
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray, Float64

rospy.init_node('publish_ur5eSpot', anonymous=True)
tf_listener_ = tf.TransformListener()

rospy.sleep(1)

(init_ur_trans, init_ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
init_ur_pose = geometry_msgs.msg.PoseStamped()
init_ur_pose.header.stamp = rospy.Time.now()
init_ur_pose.pose.position.x = init_ur_trans[0]
init_ur_pose.pose.position.y = init_ur_trans[1]
init_ur_pose.pose.position.z = init_ur_trans[2]
init_ur_pose.pose.orientation.x = init_ur_rot[0]
init_ur_pose.pose.orientation.y = init_ur_rot[1]
init_ur_pose.pose.orientation.z = init_ur_rot[2]
init_ur_pose.pose.orientation.w = init_ur_rot[3]

rate = rospy.Rate(100.0)
pub = rospy.Publisher('/spot1', geometry_msgs.msg.PoseStamped, queue_size=10)
pub_linear = rospy.Publisher('/servo_server/internal/linear_scale', Float64)

while not rospy.is_shutdown():
    linear_velocity_scale = Float64()
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = "base_link"
    target_pose.pose.position.x =  init_ur_pose.pose.position.x - 0.2
    target_pose.pose.position.y =  init_ur_pose.pose.position.y + 0.1
    target_pose.pose.position.z =  init_ur_pose.pose.position.z + 0.1
    target_pose.pose.orientation.x = 0.1227878
    target_pose.pose.orientation.y = -0.872665
    target_pose.pose.orientation.z = -0.872665
    target_pose.pose.orientation.w = 0.1227878
    pub.publish(target_pose)
    rate.sleep()