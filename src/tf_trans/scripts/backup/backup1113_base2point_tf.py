#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

# Global dictionaries to store the data
b2t_tf = None
c2p_tf = {}

def callback_base_to_tool(data):
    global b2t_tf
    b2t_tf = data.data

def callback_camera_to_points(data):
    # data.data contains [tag_id, x, y, z]
    c2p_tf[data.data[0]] = data.data[1:]

def publish_base_to_point(publisher):
    global b2t_tf, c2p_tf

    if b2t_tf is None:
        rospy.loginfo("No base to tool transform received yet.")
        return

    for tag_id, point in c2p_tf.items():
        # Perform the sum here assuming you want to add the corresponding components
        base_to_targetPoint = [
            int(tag_id),
            round(b2t_tf[0], 3) + round(point[0], 3) + 0.01,
            round(b2t_tf[1], 3) + round(point[1], 3) + 0.03,
            round(b2t_tf[2], 3) + round(point[2], 3) + 0.135 
        ]
        
        # Create the message and publish it
        transform_msg = Float32MultiArray()
        transform_msg.data = base_to_targetPoint
        publisher.publish(transform_msg)
        rospy.loginfo(f"Base2point position of each tag_id {base_to_targetPoint}")

def main():
    rospy.init_node('base2point_tf')

    # Initialize subscribers
    rospy.Subscriber("/tf_B2T", Float32MultiArray, callback_base_to_tool)
    rospy.Subscriber("/tf_C2P", Float32MultiArray, callback_camera_to_points)

    # Initialize publisher for the summed transform
    pub = rospy.Publisher('/tf_B2P', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        publish_base_to_point(pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
