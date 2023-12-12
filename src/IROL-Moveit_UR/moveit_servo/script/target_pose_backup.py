#!/usr/bin/env python3
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Float64
import numpy as np

rospy.init_node('tf_listener')
# br = tf.TransformBroadcaster()
tf_listener_ = tf.TransformListener()

rospy.sleep(1)
(init_ur_trans,init_ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
init_ur_pose = geometry_msgs.msg.PoseStamped()
init_ur_pose.header.stamp = rospy.Time.now()
init_ur_pose.pose.position.x = init_ur_trans[0]
init_ur_pose.pose.position.y = init_ur_trans[1]
init_ur_pose.pose.position.z = init_ur_trans[2]
init_ur_pose.pose.orientation.x = init_ur_rot[0]
init_ur_pose.pose.orientation.y = init_ur_rot[1]
init_ur_pose.pose.orientation.z = init_ur_rot[2]
init_ur_pose.pose.orientation.w = init_ur_rot[3]
print(init_ur_trans)
print(init_ur_rot)

rate = rospy.Rate(100.0)
pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
while not rospy.is_shutdown():
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x =  init_ur_pose.pose.position.x + 0.2
        target_pose.pose.position.y =  init_ur_pose.pose.position.y + 0.1
        target_pose.pose.position.z =  init_ur_pose.pose.position.z + 0.0
        target_pose.pose.orientation.x = -0.1830127    
        target_pose.pose.orientation.y = 0.6830127
        target_pose.pose.orientation.z = 0.6830127
        target_pose.pose.orientation.w = -0.1830127

        pub.publish(target_pose)
        # i = i + 1
        rate.sleep()

# #!/usr/bin/env python3
# import roslib
# import rospy
# import math
# import tf
# import geometry_msgs.msg
# import sensor_msgs.msg
# from std_msgs.msg import Float64, Float32MultiArray
# import numpy as np


# def callback(data):

#         # br = tf.TransformBroadcaster()
#         tf_listener_ = tf.TransformListener()

#         rospy.sleep(1)
#         (init_ur_trans,init_ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
#         init_ur_pose = geometry_msgs.msg.PoseStamped()
#         init_ur_pose.header.stamp = rospy.Time.now()
#         init_ur_pose.pose.position.x = init_ur_trans[0]
#         init_ur_pose.pose.position.y = init_ur_trans[1]
#         init_ur_pose.pose.position.z = init_ur_trans[2]
#         init_ur_pose.pose.orientation.x = init_ur_rot[0]
#         init_ur_pose.pose.orientation.y = init_ur_rot[1]
#         init_ur_pose.pose.orientation.z = init_ur_rot[2]
#         init_ur_pose.pose.orientation.w = init_ur_rot[3]

#         rate = rospy.Rate(100.0)
#         pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
#         while not rospy.is_shutdown():
#                 target_pose = geometry_msgs.msg.PoseStamped()
#                 target_pose.header.stamp = rospy.Time.now()
#                 target_pose.header.frame_id = "base_link"
#                 # target_pose.pose.position.x =  data.data[1] + 0.1
#                 # target_pose.pose.position.y =  init_ur_pose.pose.position.y + 0.1
#                 # target_pose.pose.position.z =  init_ur_pose.pose.position.z
#                 target_pose.pose.position.x =  init_ur_pose.pose.position.x + 0.1
#                 target_pose.pose.position.y =  init_ur_pose.pose.position.y
#                 target_pose.pose.position.z =  init_ur_pose.pose.position.z
#                 target_pose.pose.orientation.x = 0.1830127
#                 target_pose.pose.orientation.y = -0.6830127
#                 target_pose.pose.orientation.z = -0.6830127
#                 target_pose.pose.orientation.w = 0.1830127

#                 print(target_pose)
#                 pub.publish(target_pose)
#                 rate.sleep()
                
# def listener():
#         rospy.init_node('target_pose', anonymous=True)
#         rospy.Subscriber("/tf_B2P", Float32MultiArray, callback)
        
# if __name__ == '__main__':
#         listener()
#         rospy.spin()

