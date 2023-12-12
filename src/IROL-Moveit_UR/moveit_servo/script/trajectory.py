#!/usr/bin/python3

import rospy
import numpy as np
from geometry_msgs.msg import *
import tf
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64MultiArray
    
def pub_velocity():
    pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
    rospy.init_node('trajectory', anonymous=True)
    tf_listener_ = tf.TransformListener()
    rospy.sleep(1)
    rate = rospy.Rate(500) # 120hz
    
    target_distance_x = 0.6
    # target_distance_y = -0.1
    target_angle_z = 30
    total_time = 1.5     #  6.0 >> 0.5degree   3 >> 1.0degree   1.5 >> 2.0degree   1.0 >> 3.0degree   0.75 >> 4.0degree          
    
    linear_velocity_x = target_distance_x / total_time
    # linear_velocity_y = target_distance_y / total_time
    angular_velocity_z = np.deg2rad(target_angle_z) / total_time
  
    print(np.deg2rad(target_angle_z))
    print("linnear_velocity_x: ", linear_velocity_x)
    # print("linnear_velocity_y: ", linear_velocity_y)
    print("angular_velocity_z: ", angular_velocity_z)
    
    # distance (m) to start decelerating 
    deceleration_distance = 0.05

    # move to x direction
    (init_ur_trans,init_ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
    while not rospy.is_shutdown():
        data_to_send = TwistStamped()
        (ur_trans,ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
        
        moved_distance_x = ur_trans[0] - init_ur_trans[0]
        remaining_distance_x = target_distance_x - moved_distance_x
        
        if remaining_distance_x <= 0:
            break
        
        # check if the robot is close to the target
        if remaining_distance_x <= deceleration_distance:
            deceleration_ratio_x = remaining_distance_x / deceleration_distance
            # current_linear_velocity_x = linear_velocity_x * deceleration_ratio_x
        else:
            current_linear_velocity_x = linear_velocity_x
            
        # set velocity
        data_to_send.header.stamp = rospy.Time.now()
        data_to_send.twist.linear.x = current_linear_velocity_x
        data_to_send.twist.angular.z = angular_velocity_z
        pub.publish(data_to_send)
        rate.sleep()
    
    # # move to y direction
    # (init_ur_trans, init_ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
    # while not rospy.is_shutdown():
    #     data_to_send = TwistStamped()
    #     (ur_trans, ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
        
    #     moved_distance_y = ur_trans[1] - init_ur_trans[1]
    #     remaining_distance_y = target_distance_y - moved_distance_y
        
    #     if remaining_distance_y >= 0:
    #         break
        
    #     if abs(remaining_distance_y) <= deceleration_distance:
    #         deceleration_ratio_y = abs(remaining_distance_y) / deceleration_distance
    #         current_linear_velocity_y = linear_velocity_y * deceleration_ratio_y
    #     else:
    #         current_linear_velocity_y = linear_velocity_y
            
    #     # set velocity
    #     data_to_send.header.stamp = rospy.Time.now()
    #     data_to_send.twist.linear.y = current_linear_velocity_y
    #     data_to_send.twist.angular.z = 0
    #     pub.publish(data_to_send)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        pub_velocity()
    except rospy.ROSInterruptException:
        pass