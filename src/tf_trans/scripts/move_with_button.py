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
from unity_robotics_demo_msgs.msg import Button

class Move_with_button:
    def __init__(self):
        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.target_x_cam = 0
        self.target_y_cam = 0
        self.target_z_cam = 0
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

    def left(self, msg):
        print("left")
        now_manipulator_pose = self.group.get_current_pose()
        target_pose = geometry_msgs.msg.PoseStamped()
        robot_current_euler = mr.quaternion_to_euler(np.array([now_manipulator_pose.pose.orientation.x, 
                                                            now_manipulator_pose.pose.orientation.y,
                                                            now_manipulator_pose.pose.orientation.z,
                                                            now_manipulator_pose.pose.orientation.w]))
        R_robot = mr.eulertorotation(robot_current_euler[0], robot_current_euler[1], robot_current_euler[2])
        R_z = mr.rotx(-0.0174533)
        R_target = np.dot(R_robot, R_z)
        target_euler = mr.rotation_to_euler(R_target)
        target_quaternion = mr.euler_to_quaternion(target_euler)
        target_pose.pose.position.x = now_manipulator_pose.pose.position.x 
        target_pose.pose.position.y = now_manipulator_pose.pose.position.y
        target_pose.pose.position.z = now_manipulator_pose.pose.position.z 
        target_pose.pose.orientation.x = target_quaternion[0]
        target_pose.pose.orientation.y = target_quaternion[1]
        target_pose.pose.orientation.z = target_quaternion[2]
        target_pose.pose.orientation.w = target_quaternion[3]
        self.pub.publish(target_pose)

    def right(self, msg):
        now_manipulator_pose = self.group.get_current_pose()
        target_pose = geometry_msgs.msg.PoseStamped()
        robot_current_euler = mr.quaternion_to_euler(np.array([now_manipulator_pose.pose.orientation.x, 
                                                            now_manipulator_pose.pose.orientation.y,
                                                            now_manipulator_pose.pose.orientation.z,
                                                            now_manipulator_pose.pose.orientation.w]))
        R_robot = mr.eulertorotation(robot_current_euler[0], robot_current_euler[1], robot_current_euler[2])
        R_z = mr.rotx(0.0174533)
        R_target = np.dot(R_robot, R_z)
        target_euler = mr.rotation_to_euler(R_target)
        target_quaternion = mr.euler_to_quaternion(target_euler)
        target_pose.pose.position.x = now_manipulator_pose.pose.position.x 
        target_pose.pose.position.y = now_manipulator_pose.pose.position.y
        target_pose.pose.position.z = now_manipulator_pose.pose.position.z 
        target_pose.pose.orientation.x = target_quaternion[0]
        target_pose.pose.orientation.y = target_quaternion[1]
        target_pose.pose.orientation.z = target_quaternion[2]
        target_pose.pose.orientation.w = target_quaternion[3]
        self.pub.publish(target_pose)

    def up(self, msg):
        now_manipulator_pose = self.group.get_current_pose()
        target_pose = geometry_msgs.msg.PoseStamped()
        robot_current_euler = mr.quaternion_to_euler(np.array([now_manipulator_pose.pose.orientation.x, 
                                                            now_manipulator_pose.pose.orientation.y,
                                                            now_manipulator_pose.pose.orientation.z,
                                                            now_manipulator_pose.pose.orientation.w]))
        R_robot = mr.eulertorotation(robot_current_euler[0], robot_current_euler[1], robot_current_euler[2])
        R_x = mr.rotx(0.0174533)
        R_target = np.dot(R_robot, R_x)
        target_euler = mr.rotation_to_euler(R_target)
        target_quaternion = mr.euler_to_quaternion(target_euler)
        target_pose.pose.position.x = now_manipulator_pose.pose.position.x 
        target_pose.pose.position.y = now_manipulator_pose.pose.position.y
        target_pose.pose.position.z = now_manipulator_pose.pose.position.z 
        target_pose.pose.orientation.x = target_quaternion[0]
        target_pose.pose.orientation.y = target_quaternion[1]
        target_pose.pose.orientation.z = target_quaternion[2]
        target_pose.pose.orientation.w = target_quaternion[3]
        self.pub.publish(target_pose)

    def down(self, msg):
        now_manipulator_pose = self.group.get_current_pose()
        target_pose = geometry_msgs.msg.PoseStamped()
        robot_current_euler = mr.quaternion_to_euler(np.array([now_manipulator_pose.pose.orientation.x, 
                                                            now_manipulator_pose.pose.orientation.y,
                                                            now_manipulator_pose.pose.orientation.z,
                                                            now_manipulator_pose.pose.orientation.w]))
        R_robot = mr.eulertorotation(robot_current_euler[0], robot_current_euler[1], robot_current_euler[2])
        R_x = mr.rotx(-0.0174533)
        R_target = np.dot(R_robot, R_x)
        target_euler = mr.rotation_to_euler(R_target)
        target_quaternion = mr.euler_to_quaternion(target_euler)
        target_pose.pose.position.x = now_manipulator_pose.pose.position.x 
        target_pose.pose.position.y = now_manipulator_pose.pose.position.y
        target_pose.pose.position.z = now_manipulator_pose.pose.position.z 
        target_pose.pose.orientation.x = target_quaternion[0]
        target_pose.pose.orientation.y = target_quaternion[1]
        target_pose.pose.orientation.z = target_quaternion[2]
        target_pose.pose.orientation.w = target_quaternion[3]
        self.pub.publish(target_pose)



if __name__ == '__main__':
    rospy.init_node('move_with_button')
    move_with_button = Move_with_button()
    sub = rospy.Subscriber('/left_button', Button, move_with_button.left)
    sub = rospy.Subscriber('/right_button', Button, move_with_button.right)
    sub = rospy.Subscriber('/up_button', Button, move_with_button.up)
    sub = rospy.Subscriber('/down_button', Button, move_with_button.down)
    rospy.spin()