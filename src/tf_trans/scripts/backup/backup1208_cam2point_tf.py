#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import apriltag
import threading
# import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped


class AprilTagDetector:
    def __init__(self, tag_size):
        # ROS setup
        self.bridge = CvBridge()
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.pose_publisher = rospy.Publisher('tf_C2P', Float32MultiArray, queue_size=10)
        
        self.color_image = None
        self.depth_image = None
        self.tag_size = tag_size
        self.detector = apriltag.Detector()
        
        # define camera intrinsics
        self.fx = 638.3863525390625
        self.fy = 636.7418212890625
        self.ppx = 630.9259643554688
        self.ppy = 365.64129638671875        
        # rospy.loginfo('AprilTag Detector running...')
        
        self.selected_tag_id = None
        self.selected_pose_publisher = rospy.Publisher('tf_C2P_selected', Float32MultiArray, queue_size=10)
        
    def color_callback(self, data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray_image)
            for r in results:
                self.process_result(r, color_image, self.depth_image)
                
            cv2.imshow("AprilTag Detector", color_image)
            cv2.waitKey(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                
        except CvBridgeError as e:
            rospy.logerr("Could not convert color image. Error: %s", str(e))
        
    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            rospy.logerr("Could not convert depth image. Error: %s", str(e))
            
    def run(self):
        rospy.spin()
        
    def set_selected_tag_id(self, ):
        while not rospy.is_shutdown():
            try:
                self.selected_tag_id = int(input("Enter the tag's ID: "))
                rospy.loginfo(f"Selected tag ID: {self.selected_tag_id}")
            except ValueError:
                rospy.logwarn("Invalid tag ID. Please try again.")

    def process_result(self, r, color_image, depth_image):
        # extract the bounding box (x, y)-coordinates for the AprilTag
        (ptA, ptB, ptC, ptD) = r.corners
        ptA = (int(ptA[0]), int(ptA[1]))
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        cv2.line(color_image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(color_image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(color_image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(color_image, ptD, ptA, (0, 255, 0), 2)
        
        # center of april tag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)
        
        # place the tag ID on the image
        cv2.putText(color_image, str(r.tag_id), (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # calculate the pose of the april tag
        camera_params = (self.fx, self.fy, self.ppx, self.ppy)
        pose, e0, e1 = self.detector.detection_pose(r, camera_params, tag_size = self.tag_size)
        
        # print(f"ID: {r.tag_id} pose:\n {pose}")
        global_cx = round(pose[0][3], 3)
        global_cy = round(pose[2][3], 3)
        global_cz = -round(pose[1][3], 3)

        # Publish the position as a ROS topic
        pose_msg = Float32MultiArray()
        pose_msg.data = [int(r.tag_id), round(global_cx, 3), round(global_cy, 3), round(global_cz, 3)]
        self.pose_publisher.publish(pose_msg)
        # rospy.loginfo(f"ID: {r.tag_id}, C2P_Position: {global_cx, global_cy, global_cz}")
        
        if self.selected_tag_id is not None and r.tag_id == self.selected_tag_id:
            # Publish the position as a ROS topic
            selected_pose_msg = Float32MultiArray()
            selected_pose_msg.data = [int(r.tag_id), round(global_cx, 3), round(global_cy, 3), round(global_cz, 3)]
            self.selected_pose_publisher.publish(selected_pose_msg)
            rospy.loginfo(f"ID: {r.tag_id}, C2P_Position: {global_cx, global_cy, global_cz}")
        
def main():
    rospy.init_node('cam2point_tf')
    tag_size = 0.04  # Replace with your actual tag size
    detector = AprilTagDetector(tag_size)
    input_thread = threading.Thread(target=detector.set_selected_tag_id)
    input_thread.start()
    
    try:
        detector.run()
    except rospy.ROSInterruptException:
        pass
    
if __name__ == '__main__':
    main()
    