#!/usr/bin/python3
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
import numpy as np
from geometry_msgs.msg import *
import tf

def pub_velocity():
    pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
    rospy.init_node('trajectory', anonymous=True)
    tf_listener_ = tf.TransformListener()
    rospy.sleep(1)
    rate = rospy.Rate(500) # 120hz
    total_time = 1.5     #  6--> 0.5 degree      3 --> 1degree     1.5 --> 2degree        1 --> 3degree             0.75 --> 4 degree          

    # Time step (in seconds)
    # time_step = 500
    
    angular_velocity_z = np.deg2rad(30) / total_time
    linnear_velocity_x = 0.6 / total_time
    
    print(np.deg2rad(30))
    print("angular_velocity_z: ", angular_velocity_z)
    print("linnear_velocity_x: ", linnear_velocity_x)

    (init_ur_trans,init_ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
    i = 0

    while not rospy.is_shutdown():
        data_to_send = TwistStamped()
        (ur_trans,ur_rot) = tf_listener_.lookupTransform('/base_link', '/tool0', rospy.Time(0))
        # print(init_ur_trans[0] - ur_trans[0])
        if 0.56 > abs(init_ur_trans[0] -ur_trans[0]):
            data_to_send.header.stamp = rospy.Time.now()
            data_to_send.twist.linear.x = linnear_velocity_x
            data_to_send.twist.linear.y = 0
            data_to_send.twist.linear.z = 0
            data_to_send.twist.angular.x = 0
            data_to_send.twist.angular.y = 0
            data_to_send.twist.angular.z = angular_velocity_z

            pub.publish(data_to_send)
            # print(data_to_send)
        else:
            data_to_send.header.stamp = rospy.Time.now()
            data_to_send.twist.linear.x = 0
            data_to_send.twist.linear.y = 0
            data_to_send.twist.linear.z = 0
            data_to_send.twist.angular.x = 0
            data_to_send.twist.angular.y = 0
            data_to_send.twist.angular.z = 0
            pub.publish(data_to_send)
            # print(data_to_send)
        i = i + 1
        rate.sleep()

if __name__ == '__main__':
    try:
        pub_velocity()
    except rospy.ROSInterruptException:
        pass