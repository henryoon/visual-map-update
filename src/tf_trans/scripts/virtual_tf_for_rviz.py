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

       position.header.frame_id = "virtual_tf"

       position.pose.position.x = 0
       position.pose.position.y = 0

       position.pose.orientation.w = 1

       self.position = position
       self.pub = rospy.Publisher("virtual_tf", PoseStamped, queue_size=1)

   def publishPose(self):
       self.position.header.stamp = rospy.Time.now()

       self.pub.publish(self.position)


if __name__ == '__main__':
   rospy.init_node("virtual_tf")

   virtual = TF()

   r = rospy.Rate(1)
   tf_broadcaster = tf.TransformBroadcaster()
   euler_angle = [0,30,90]
   while not rospy.is_shutdown():

        tf_broadcaster.sendTransform(
            translation=[0, -2, 2],
            rotation=euler_to_quaternion(euler_angle),
            time=rospy.Time.now(),
            child="virtual_tf",
            parent="base_link"
        )

        virtual.publishPose()

        r.sleep()
