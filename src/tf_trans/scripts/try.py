#!/usr/bin/env python3

import rospy
import tf

rospy.init_node('only_read_tf')
listener = tf.TransformListener()
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('base_link', 'tool0', rospy.Time(0))
        print("tool0 link translation: (%.2f, %.2f, %.2f)" % (trans[0], trans[1], trans[2]))
        print("tool0 link rotation: (%.2f, %.2f, %.2f, %.2f)" % (rot[0], rot[1], rot[2], rot[3]))
        # rospy.loginfo("tool0 Link Translation: (%.2f, %.2f, %.2f)" % (trans[0], trans[1], trans[2]))
        # rospy.loginfo("tool0 Link Rotation: (%.2f, %.2f, %.2f, %.2f)" % (rot[0], rot[1], rot[2], rot[3]))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    rate.sleep()