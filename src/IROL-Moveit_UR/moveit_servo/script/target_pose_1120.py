#!/usr/bin/env python3
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import sensor_msgs.msg
from std_msgs.msg import Float64
import numpy as np
from tf.transformations import *

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

def get_current_pose():
        try:
                (trans, rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
                return trans, rot, rospy.Time.now()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("TF Exception")
                return None, None, None
        
def calculate_speed(prev_pose, current_pose, prev_time, current_time):
        if prev_pose is None or current_pose is None:
                return None
        
        delta_distance = np.linalg.norm(np.array(current_pose) - np.array(prev_pose))
        delta_time = (current_time - prev_time).to_sec()
        
        if delta_time > 0:
                return delta_distance / delta_time
        else:
                return None
        
prev_pose, prev_time = None, None
                
def calculate_angular_speed(prev_rot, current_rot, prev_time, current_time):
        if prev_rot is None or current_rot is None:
                return None
        
        # Convert quaternions to Euler angles
        prev_euler = euler_from_quaternion(prev_rot)
        current_euler = euler_from_quaternion(current_rot)
        
        # Calculate angular change in each axis
        delta_angles = np.array(current_euler) - np.array(prev_euler)
        delta_time = (current_time - prev_time).to_sec()

        if delta_time > 0:
                # Compute angular speed (rad/s)
                angular_speed = delta_angles / delta_time
                return angular_speed
        else:
                return None
        
prev_rot, prev_time = None, None

rate = rospy.Rate(100.0)
pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

while not rospy.is_shutdown():
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x =  init_ur_pose.pose.position.x + 0.65
        target_pose.pose.position.y =  init_ur_pose.pose.position.y - 0.05
        target_pose.pose.position.z =  init_ur_pose.pose.position.z + 0.0
        target_pose.pose.orientation.x = 0.1227878    # 0.092296      
        target_pose.pose.orientation.y = -0.6963642    # -0.7010574
        target_pose.pose.orientation.z = -0.6963642    # -0.7010574
        target_pose.pose.orientation.w = 0.1227878

        pub.publish(target_pose)
        
        current_pose, current_rot, current_time = get_current_pose()
        
        if prev_pose is not None and prev_rot is not None:
                speed = calculate_speed(prev_pose, current_pose, prev_time, current_time)
                angular_speed = calculate_angular_speed(prev_rot, current_rot, prev_time, current_time)

                # Check if the linear speed is not zero before logging
                if speed is not None and speed > 0:
                        rospy.loginfo("Linear Speed (m/s): %f", speed)

                # Check if the angular speed is not zero before logging
                if angular_speed is not None and np.any(angular_speed > 0):
                        rospy.loginfo("Angular Speed (rad/s): x: %f, y: %f, z: %f", *angular_speed)


        prev_pose, prev_rot, prev_time = current_pose, current_rot, current_time
        
        rate.sleep()