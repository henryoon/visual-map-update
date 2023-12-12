#!/usr/bin/env python3

import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import moveit_commander
from geometry_msgs.msg import *
import Modern_Robotics_lib as mr
import numpy as np
from std_msgs.msg import Float32MultiArray

class Target_point:
    def __init__(self):
        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.target_x_cam = 0
        self.target_y_cam = 0
        self.target_z_cam = 0
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

    def target_cam(self, msg):
        (trans, rot) = self.listener.lookupTransform('base_link', 'tool0', rospy.Time(0))
        print(trans)
        now_manipulator_pose = self.group.get_current_pose()
        print(now_manipulator_pose.pose.position)
        print(now_manipulator_pose.pose.orientation)
        target_pose = geometry_msgs.msg.PoseStamped()
        robot_current_ori = mr.quaternion_to_euler(np.array([now_manipulator_pose.pose.orientation.x, 
                                                            now_manipulator_pose.pose.orientation.y,
                                                            now_manipulator_pose.pose.orientation.z,
                                                            now_manipulator_pose.pose.orientation.w]))
        tool_to_cam = np.array([[0.05], [0.121], [0.017]])
        tool_to_fork = np.array([[0], [-0.06], [0.35]])
        target = np.array([[msg.data[0]*-0.001], [msg.data[1]*0.001], [msg.data[2]*0.001]]) + tool_to_cam - tool_to_fork
        R_robot = mr.eulertorotation(robot_current_ori[0], robot_current_ori[1], robot_current_ori[2])
        base_to_tool = np.array([[trans[0]], [trans[1]], [trans[2]]])
        target_point = np.dot(R_robot, target)
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "base_link"
        # target_pose.pose.position.x = now_manipulator_pose.pose.position.x + msg.data[0]*0.001
        # target_pose.pose.position.y = now_manipulator_pose.pose.position.y 
        # target_pose.pose.position.z = trans[2] + msg.data[1]*0.001
        target_pose.pose.position.x = target_point[0] + base_to_tool[0]
        target_pose.pose.position.y = target_point[1] + base_to_tool[1]
        target_pose.pose.position.z = target_point[2] + base_to_tool[2]
        target_pose.pose.orientation.x = now_manipulator_pose.pose.orientation.x
        target_pose.pose.orientation.y = now_manipulator_pose.pose.orientation.y
        target_pose.pose.orientation.z = now_manipulator_pose.pose.orientation.z
        target_pose.pose.orientation.w = now_manipulator_pose.pose.orientation.w
        print(target_pose.pose.position)
        self.pub.publish(target_pose)



if __name__ == '__main__':
    rospy.init_node('read_tf')
    target_point = Target_point()
    sub = rospy.Subscriber('/R_coordinate', Float32MultiArray,target_point.target_cam)
    rospy.spin()