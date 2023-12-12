#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import tf.transformations as tf_trans

# Global dictionaries to store the data
b2t_tf = None
c2p_tf = {}
selected_c2p_tf = None  # Added for selected tag

def callback_base_to_tool(data):
    global b2t_tf
    b2t_tf = data.data

def callback_camera_to_points(data):
    c2p_tf[data.data[0]] = data.data[1:]

def callback_camera_to_selected_point(data):
    global selected_c2p_tf
    # data.data contains [tag_id, x, y, z]
    selected_c2p_tf = data.data

def publish_base_to_point(publisher):
    global b2t_tf, c2p_tf
    # Existing logic...

def publish_base_to_selected_point(publisher):
    global b2t_tf, selected_c2p_tf

    if b2t_tf is None or selected_c2p_tf is None:
        # rospy.loginfo("No base to tool or selected point transform received yet.")
        return
    
    # # translation + rotation
    # translation = b2t_tf[:3]
    # rotation_quat = b2t_tf[3:]
    # # Convert the quaternion to a rotation matrix
    # rotation_matrix = tf_trans.quaternion_matrix(rotation_quat)
    # # create a 4x1 matrix for the selected point
    # selected__point = [selected_c2p_tf[1], selected_c2p_tf[2], selected_c2p_tf[3], 1]
    # # apply rotation and translation to the selected point
    # rotated_point = tf_trans.translation_matrix(selected__point)
    # rotated_point = tf_trans.concatenate_matrices(rotation_matrix, rotated_point)
    # rotated_point = tf_trans.translation_from_matrix(rotated_point)
    # transformed_point = [rotated_point[0] + translation[0], 
    #                      rotated_point[1] + translation[1], 
    #                      rotated_point[2] + translation[2]]
    # base_to_selectedPoint = [int(selected_c2p_tf[0])] + transformed_point
    
    # translation
    # Perform the sum here assuming you want to add the corresponding components
    base_to_selectedPoint = [
        int(selected_c2p_tf[0]),
        round(b2t_tf[0] + selected_c2p_tf[1], 3)+0.03,
        round(b2t_tf[1] + selected_c2p_tf[2], 3)+0.03,
        round(b2t_tf[2] + selected_c2p_tf[3], 3)+0.18
    ]
    
    # Create the message and publish it
    transform_msg = Float32MultiArray()
    transform_msg.data = base_to_selectedPoint
    publisher.publish(transform_msg)
    # rospy.loginfo(f"ID: {base_to_selectedPoint[0]}, B2P_Position: {base_to_selectedPoint[1:]}")


def main():
    rospy.init_node('base2point_tf_selected')

    # Initialize subscribers
    rospy.Subscriber("/tf_B2T", Float32MultiArray, callback_base_to_tool)
    rospy.Subscriber("/tf_C2P", Float32MultiArray, callback_camera_to_points)
    rospy.Subscriber("/tf_C2P_selected", Float32MultiArray, callback_camera_to_selected_point)  # New subscriber

    # Initialize publishers
    pub = rospy.Publisher('/tf_B2P', Float32MultiArray, queue_size=10)
    pub_selected = rospy.Publisher('/tf_B2P_selected', Float32MultiArray, queue_size=10)  # New publisher

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        publish_base_to_point(pub)
        publish_base_to_selected_point(pub_selected)  # Publish selected points
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
