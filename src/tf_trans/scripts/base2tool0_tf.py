#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import String, Float32MultiArray

rospy.init_node('base2tool0_tf')

listener = tf.TransformListener()
pose_publisher = rospy.Publisher('tf_B2T', Float32MultiArray, queue_size=10)
rate = rospy.Rate(10.0)

while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('base_link', 'tool0', rospy.Time(0))
        # rospy.loginfo("tool0 Link Translation: (%.3f, %.3f, %.3f)" % (trans[0], trans[1], trans[2]))
        # rospy.loginfo("tool0 Link Rotation: (%.3f, %.3f, %.3f, %.3f)" % (rot[0], rot[1], rot[2], rot[3]))
        
        transform_data = list(trans) + list(rot)
        transform_msg = Float32MultiArray()
        transform_msg.data = transform_data
        pose_publisher.publish(transform_msg)
        # translation_msg = Float32MultiArray()
        # translation_msg.data = trans
        # pose_publisher.publish(translation_msg)
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    
    rate.sleep()