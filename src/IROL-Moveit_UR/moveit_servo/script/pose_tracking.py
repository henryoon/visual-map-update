#!/usr/bin/env python3

from cmath import pi
from tokenize import Pointfloat
import rospy
import sys
from sensor_msgs.msg import Imu
from std_msgs.msg import *
import moveit_msgs.msg
import numpy as np
import math
from geometry_msgs.msg import *
from tf.transformations import *
import time
import tf
from collections import deque
import moveit_commander
# from mtx_driver.msg import imu_msg
import copy
from moveit_commander.conversions import pose_to_list

class Pub_target_pose:
    def __init__(self):
        rospy.init_node('pose_tracking')
        tf_listener_ = tf.TransformListener()
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        # self.init_pose = self.group.get_current_pose()
        (self.init_ur_trans, self.init_ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
        self.pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
    def callback(self, msg):
        now_manipulator_pose = self.group.get_current_pose()
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "base_link"
        # target_pose.pose.position.x =  now_manipulator_pose.pose.position.x #- msg.position.y
        # target_pose.pose.position.y =  now_manipulator_pose.pose.position.y #+ msg.position.x
        # target_pose.pose.position.z =  now_manipulator_pose.pose.position.z #+ msg.position.z
        # target_pose.pose.orientation.x = now_manipulator_pose.pose.orientation.x #+ msg.orientation.x
        # target_pose.pose.orientation.y = now_manipulator_pose.pose.orientation.y #+ msg.orientation.y
        # target_pose.pose.orientation.z = now_manipulator_pose.pose.orientation.z #+ msg.orientation.z
        # target_pose.pose.orientation.w = now_manipulator_pose.pose.orientation.w #+ msg.orientation.w
        # target_pose.pose.position.x =  self.init_pose.pose.position.x #- msg.position.y
        # target_pose.pose.position.y =  self.init_pose.pose.position.y #+ msg.position.x
        # target_pose.pose.position.z =  self.init_pose.pose.position.z #+ msg.position.z
        # target_pose.pose.orientation.x = self.init_pose.pose.orientation.x #+ msg.orientation.x
        # target_pose.pose.orientation.y = self.init_pose.pose.orientation.y #+ msg.orientation.y
        # target_pose.pose.orientation.z = self.init_pose.pose.orientation.z #+ msg.orientation.z
        # target_pose.pose.orientation.w = self.init_pose.pose.orientation.w #+ msg.orientation.w
        print(target_pose)
        # self.pub.publish(target_pose)




if __name__ == '__main__':
    pub_target_pose = Pub_target_pose()
    sub = rospy.Subscriber('target/pose', Pose, pub_target_pose.callback)
    rospy.spin()
    
# br = tf.TransformBroadcaster()
# tf_listener_ = tf.TransformListener()
# pi = math.pi
# # rospy.sleep(1)
# # (init_body_trans,init_body_rot) = tf_listener_.lookupTransform('/shoulder', '/wrist', rospy.Time(0))
# rospy.sleep(1)
# (init_ur_trans,init_ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
# init_ur_pose = geometry_msgs.msg.PoseStamped()
# init_ur_pose.header.stamp = rospy.Time.now()
# init_ur_pose.pose.position.x = init_ur_trans[0]
# init_ur_pose.pose.position.y = init_ur_trans[1]
# init_ur_pose.pose.position.z = init_ur_trans[2]
# init_ur_pose.pose.orientation.x = init_ur_rot[0]
# init_ur_pose.pose.orientation.y = init_ur_rot[1]
# init_ur_pose.pose.orientation.z = init_ur_rot[2]
# init_ur_pose.pose.orientation.w = init_ur_rot[3]
# sub = rospy.Subscriber('target_pose_dh', Pose, callback)
# pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
# rospy.spin()
# shoulder - wrist

# base_link - tool0