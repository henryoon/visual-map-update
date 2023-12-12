#!/usr/bin/env python3

import rospy
import tf
import moveit_commander
from geometry_msgs.msg import PoseStamped
import sys
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler
from math import radians

b2p_tf = {}

def callback_base_to_point(data):
    # data.data contains [tag_id, x, y, z]
    b2p_tf[data.data[0]] = data.data[1:]
    
def initialize_moveit(): 
    # Initialize MoveIT
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    return move_group

def calculate_camPose(width, distance, height, angle):
    global b2p_tf
    
    if b2p_tf is None:
        rospy.loginfo("No base to point transform received yet.")
        return
    
    for tag_id, point in b2p_tf.items():
        target_point = [tag_id, point[0], point[1], point[2]]
    
    # Calculate the camera position
    cam_pose = PoseStamped()
    cam_pose.header.frame_id = "base_link"
    cam_pose.pose.position.x = target_point[1] - width    # 0.577
    cam_pose.pose.position.y = target_point[2] - distance # 1.0
    cam_pose.pose.position.z = target_point[3] + height   # 0.15
    
    # convert the 30 degree angle to quaternion
    q = quaternion_from_euler(0, 0, radians(angle))
    cam_pose.pose.orientation.x = q[0]
    cam_pose.pose.orientation.y = q[1]
    cam_pose.pose.orientation.z = q[2]
    cam_pose.pose.orientation.w = q[3]
    
    return cam_pose

def move_to_spot(move_group, pose):
    move_group.set_pose_target(pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

def main():
    # ROS node initialization
    rospy.init_node('move_ur5e_to_spot')
    
    rospy.Subscriber("/tf_B2P", PoseStamped, callback_base_to_point)
    rospy.wait_for_message("/tf_B2P", PoseStamped)

    move_group = initialize_moveit()
    
    width = 0.577
    distance = 1.0
    height = 0.15
    
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        if b2p_tf:
            cam_pose = calculate_camPose(width, distance, height, 30)
            if cam_pose is not None:
                move_to_spot(move_group, cam_pose)
            else:
                rospy.loginfo("No transform data received yet.")
    
        moveit_commander.roscpp_shutdown()
           
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
# ## ver2
# def callback(data):
#     # calculate the position of the camera
#     # if data.data[1] <= 0:
#     #     spot_x = data.data[1] + 0.577
#     # else:
#     #     spot_x = data.data[1] - 0.577

#     spot_x = data.data[1] + 0.577
#     spot_y = data.data[2] - 1.0
#     spot_z = data.data[3] + 0.15
    
#     # create new pose message
#     spot_pose = Pose()
#     spot_pose.position.x = spot_x
#     spot_pose.position.y = spot_y
#     spot_pose.position.z = spot_z
    
#     # orientation
#     spot_pose.orientation.x = 0.1227878
#     spot_pose.orientation.y = -0.872665
#     spot_pose.orientation.z = -0.872665
#     spot_pose.orientation.w = 0.1227878
    
#     pub.publish(spot_pose)
    
# def listener():
#     rospy.init_node('move_ur5e_to_spot', anonymous=True)
#     rospy.Subscriber("/tf_B2P", Float32MultiArray, callback)
    
#     global pub
#     pub = rospy.Publisher('/tf_spot1', Pose, queue_size=10)
#     rospy.spin()
    
# if __name__ == '__main__':
#     listener()