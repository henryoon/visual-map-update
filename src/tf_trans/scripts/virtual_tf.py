#!/usr/bin/python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[2] * math.pi / 180, r[1] * math.pi / 180, r[0] * math.pi / 180)
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return np.array([qx, qy, qz, qw])

class TF(object):
   def __init__(self):
       position = PoseStamped()

       position.header.frame_id = "real_tag_0"

       position.pose.position.x = 0
       position.pose.position.y = 0

       position.pose.orientation.w = 1

       self.position = position
       self.pub = rospy.Publisher("real_tag_0", PoseStamped, queue_size=1)
       
       position_fork = PoseStamped()

if __name__ == '__main__':
   rospy.init_node("virtual_tf")

   virtual = TF()

   r = rospy.Rate(300)
   tf_broadcaster = tf.TransformBroadcaster()
   while not rospy.is_shutdown():

        tf_broadcaster.sendTransform(
            translation=[0.05, 0.161, 0.01],
            rotation=[-0.5, -0.5, -0.5, 0.5],
            time=rospy.Time.now(),
            child="camera_link",
            parent="tool0"
        )
        # tf_broadcaster.sendTransform(
        #     translation=[0,0,0],
        #     rotation=[0, 1, 0, 0],
        #     time=rospy.Time.now(),
        #     child="real_tag_0",
        #     parent="tag_0"
        # )
        # tf_broadcaster.sendTransform(
        #     translation=[0,-0.06,0.35],
        #     rotation=[0, 0, 0, 1],
        #     time=rospy.Time.now(),
        #     child="fork",
        #     parent="tool0"
        # )

        # virtual.publishPose()
        # virtual.publishPose()

        r.sleep()
