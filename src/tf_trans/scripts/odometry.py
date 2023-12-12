#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def quaternion_to_euler(r):
        (x, y, z, w) = (r[0], r[1], r[2], r[3])
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x * 180 / math.pi, pitch_y* 180 / math.pi, yaw_z* 180 / math.pi

rospy.init_node('odometry')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

vx = 0
vy = 0
vth = 0

listener = tf.TransformListener()
current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(500)
while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/base_link_inertia', '/shoulder_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    current_time = rospy.Time.now()
    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (0, 0, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time

    # print(quaternion_to_euler(rot))
    r.sleep()

