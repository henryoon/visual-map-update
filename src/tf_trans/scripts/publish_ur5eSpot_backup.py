#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

def callback(data):
    # calculate the position of the camera
    # if data.data[1] <= 0:
    #     spot_x = data.data[1] + 0.577
    # else:
    #     spot_x = data.data[1] - 0.577

    spot_x = data.data[1] + 0.577
    spot_y = data.data[2] - 1.0
    spot_z = data.data[3] + 0.15
    
    # position
    spot_pose = Pose()
    spot_pose.position.x = spot_x
    spot_pose.position.y = spot_y
    spot_pose.position.z = spot_z
    
    # orientation
    spot_pose.orientation.x = 0.1227878
    spot_pose.orientation.y = -0.872665
    spot_pose.orientation.z = -0.872665
    spot_pose.orientation.w = 0.1227878
    
    pub.publish(spot_pose)
    
def listener():
    rospy.init_node('publish_ur5eSpot', anonymous=True)
    rospy.Subscriber("/tf_B2P", Float32MultiArray, callback)
    
    global pub
    pub = rospy.Publisher('/tf_spot1', Pose, queue_size=10)
    # rospy.loginfo(f"publishing spot1 pose {pub}")
    rospy.spin()
    
if __name__ == '__main__':
    listener()