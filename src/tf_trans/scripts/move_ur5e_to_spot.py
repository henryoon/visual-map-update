#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

def move_to_pose(pose):
    move_group.set_pose_target(pose)
    plan = move_group.plan()
    move_group.execute(plan, wait=True)
    
def callback(data):
    spot_pose = Pose()
    spot_pose.position.x = data.data[0]
    spot_pose.position.y = data.data[1]
    spot_pose.position.z = data.data[2]

    move_to_pose(spot_pose)
    
def listener():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5e_to_spot', anonymous=True)
    global move_group
    move_group = moveit_commander.MoveGroupCommander("manipulator") 
    
    rospy.Subscriber("/tf_spot1", Pose, callback)
    pub = rospy.Publisher('/move_to_spot', geometry_msgs.msg, queue_size=10)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
    
    #!/usr/bin/env python3
# import roslib
# import rospy
# import math
# import tf
# import geometry_msgs.msg
# import sensor_msgs.msg
# from std_msgs.msg import Float64
# import numpy as np


# rospy.init_node('tf_listener')
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
# print(init_ur_trans)
# print(init_ur_rot)

# i = 0
# rate = rospy.Rate(100.0)
# pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
# pub_linear = rospy.Publisher('/servo_server/internal/linear_scale', Float64)
# while not rospy.is_shutdown():
#         # print(target_move)[ 0.1227878, -0.6963642, -0.6963642, 0.1227878 ]
#         linear_velocity_scale = Float64()
#         target_pose = geometry_msgs.msg.PoseStamped()
#         target_pose.header.stamp = rospy.Time.now()
#         target_pose.header.frame_id = "base_link"
#         target_pose.pose.position.x =  init_ur_pose.pose.position.x - 0.2
#         target_pose.pose.position.y =  init_ur_pose.pose.position.y + 0.1
#         target_pose.pose.position.z =  init_ur_pose.pose.position.z + 0.1
#         target_pose.pose.orientation.x = 0.1227878
#         target_pose.pose.orientation.y = -0.872665
#         target_pose.pose.orientation.z = -0.872665
#         target_pose.pose.orientation.w = 0.0
#         # if i < 1000:

#         #     target_pose.pose.position.x =  -0.13
#         #     target_pose.pose.position.y =  0.42 
#         #     target_pose.pose.position.z = 0.4
#         #     target_pose.pose.orientation = init_ur_pose.pose.orientation
#         #     print("move")
#         # # else:
#         # #     target_pose.pose.position.x =  init_ur_pose.pose.position.x
#         # #     target_pose.pose.position.y =  init_ur_pose.pose.position.y
#         # #     target_pose.pose.position.z =  init_ur_pose.pose.position.z 
#         # #     target_pose.pose.orientation = init_ur_pose.pose.orientation
#         # #     print("stop")
#         # linear_velocity_scale.data = 0.8
#         # pub_linear.publish(linear_velocity_scale)
#         pub.publish(target_pose)
#         i = i + 1
#         rate.sleep()


# # # shoulder - wrist

# # # base_link - tool0