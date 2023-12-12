#!/usr/bin/python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped




if __name__ == '__main__':
    rospy.init_node("tf_trans")
    br = tf.TransformBroadcaster()
    br.sendTransform(
        translation=[0, 0, 0],
        rotation=[0., 0., 0., 1],
        time=rospy.Time.now(),
        child="camera_link",
        parent="base_link"
    )
    print(br)
    rospy.spin()