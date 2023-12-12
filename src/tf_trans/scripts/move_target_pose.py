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

    def target_cam(self):
        (trans, rot) = self.listener.lookupTransform('wrist_1_link', 'real_tag_0', rospy.Time(0))
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

move = 0
target_position = [0,0,0]
target_euler = [0,0,0]

def target_correct(target_position, target_orientation, current_position, current_orientation):
    global move
    print(target_position, current_position)
    print(target_orientation, current_orientation)
    if current_orientation[0] -0.05 <= target_orientation[0] <= current_orientation[0] + 0.05:
        if current_orientation[1] -0.05 <= target_orientation[1] <= current_orientation[1] + 0.05:
            if current_orientation[2] -0.05 <= target_orientation[2] <= current_orientation[2] + 0.05:
                if current_position[0] - 0.05 <= target_position[0] <= current_position[0] + 0.05:
                    if current_position[1] - 0.05 <= target_position[1] <= current_position[1] + 0.05:
                        if current_position[2] - 0.05 <= target_position[2] <= current_position[2] + 0.05:
                            move = move + 1

if __name__ == '__main__':
    rospy.init_node('move_target_pose')
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
    target_pose = geometry_msgs.msg.PoseStamped()
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now_manipulator_pose = group.get_current_pose()
            (tag_trans, tag_rot) = listener.lookupTransform('base_link', 'real_tag_0', rospy.Time(0))
            (fork_trans, fork_rot) = listener.lookupTransform('fork', 'real_tag_0', rospy.Time(0))
            robot_current_euler = mr.quaternion_to_euler(np.array([now_manipulator_pose.pose.orientation.x,
                                                                 now_manipulator_pose.pose.orientation.y,
                                                                 now_manipulator_pose.pose.orientation.z,
                                                                 now_manipulator_pose.pose.orientation.w]))  
            R_robot = mr.eulertorotation(robot_current_euler[0], robot_current_euler[1], robot_current_euler[2])
            robot_current_position = np.array([now_manipulator_pose.pose.position.x,
                                               now_manipulator_pose.pose.position.y,
                                               now_manipulator_pose.pose.position.z])           
            target_correct(np.round(target_position, 2), np.round(target_euler, 2), np.round(robot_current_position, 2), np.round(robot_current_euler, 2))
            if move == 0:
                print('1')
                target_pose.pose.position.x = tag_trans[0]
                target_pose.pose.position.y = now_manipulator_pose.pose.position.y
                target_pose.pose.position.z = tag_trans[2] - 0.095 + 0.053
                target_pose.pose.orientation.x = tag_rot[0]
                target_pose.pose.orientation.y = tag_rot[1]
                target_pose.pose.orientation.z = tag_rot[2]
                target_pose.pose.orientation.w = tag_rot[3]
                target_position = np.array([tag_trans[0], now_manipulator_pose.pose.position.y, tag_trans[2]- 0.095 + 0.053])
                target_euler = mr.quaternion_to_euler(tag_rot)
                print(target_euler)
                pub.publish(target_pose)
                move = 1 
            if move == 2:
                print('2')
                before_target_position_2 = np.array([fork_trans[0], 0 , fork_trans[2]+0.25])
                after_target_position_2 = np.dot(R_robot, before_target_position_2)
                
                target_pose.pose.position.x = now_manipulator_pose.pose.position.x + after_target_position_2[0]
                target_pose.pose.position.y = now_manipulator_pose.pose.position.y + after_target_position_2[1]
                target_pose.pose.position.z = now_manipulator_pose.pose.position.z + after_target_position_2[2]
                target_pose.pose.orientation.x = now_manipulator_pose.pose.orientation.x
                target_pose.pose.orientation.y = now_manipulator_pose.pose.orientation.y
                target_pose.pose.orientation.z = now_manipulator_pose.pose.orientation.z
                target_pose.pose.orientation.w = now_manipulator_pose.pose.orientation.w
                target_position = np.array([now_manipulator_pose.pose.position.x + after_target_position_2[0], now_manipulator_pose.pose.position.y + after_target_position_2[1], now_manipulator_pose.pose.position.z + after_target_position_2[2]])
                target_euler = mr.quaternion_to_euler(np.array([now_manipulator_pose.pose.orientation.x,
                                                                 now_manipulator_pose.pose.orientation.y,
                                                                 now_manipulator_pose.pose.orientation.z,
                                                                 now_manipulator_pose.pose.orientation.w]))  
                pub.publish(target_pose)
                move = 3
            if move == 4:
                R_x = mr.rotx(-0.349066)
                R_target = np.dot(R_robot, R_x)
                target_euler = mr.rotation_to_euler(R_target)
                target_quaternion = mr.euler_to_quaternion(target_euler)
                print('3')
                target_pose.pose.position.x = now_manipulator_pose.pose.position.x 
                target_pose.pose.position.y = now_manipulator_pose.pose.position.y
                target_pose.pose.position.z = now_manipulator_pose.pose.position.z + 0.15 
                target_pose.pose.orientation.x = target_quaternion[0]
                target_pose.pose.orientation.y = target_quaternion[1]
                target_pose.pose.orientation.z = target_quaternion[2]
                target_pose.pose.orientation.w = target_quaternion[3]
                pub.publish(target_pose)
                move = 5

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()