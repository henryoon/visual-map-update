#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import apriltag
import math
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class AprilTagDetector:
    def __init__(self, tag_size = 0.077):
        # rospy.init_node('apriltag_position', anonymous=True)
        # self.pose_publisher = rospy.Publisher('/apriltag_pose', PoseStamped, queue_size=10)
        
        # Configure depth and color streams
        self.tag_size = tag_size
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Confguring depth and color streams
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start the camera pipeline
        self.profile = self.pipeline.start(self.config)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
        self.color_intrinsics = color_profile.get_intrinsics()

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        print("Depth Scale is: ", self.depth_scale)

        # Get the intrinsic parameters of the depth sensor
        # intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        # Create an AprilTag detector object
        self.detector = apriltag.Detector()
        tag_size = 0.077  # Replace with your actual tag size
    
    def run(self):
        try:
            while True:
                # Wait for a coherent pair of frames: depth and color
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                # Convert the color image to grayscale
                gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                # Detect AprilTags in the grayscale image
                results = self.detector.detect(gray_image)

                # Loop over the detections
                for r in results:
                    self.process_result(r, color_image, depth_frame)
                    
                # cv2.imshow('RealSense', color_image)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            
    def process_result(self, r, color_image, depth_frame):
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
        camera_params = (self.color_intrinsics.fx, self.color_intrinsics.fy, self.color_intrinsics.ppx, self.color_intrinsics.ppy)
        pose, e0, e1 = self.detector.detection_pose(r, camera_params, tag_size = self.tag_size)
        
        global_cx = round(pose[0][3], 3)
        global_cy = round(pose[2][3], 3)
        global_cz = -round(pose[1][3], 3)
        # display only up to 3 decimal places
        # print(f"ID: {r.tag_id}, Position: {round(pose[0][3], 3), round(pose[1][3], 3), round(pose[2][3], 3)}")
        print(f"ID: {r.tag_id}, Position: {global_cx, global_cy, global_cz}")
        
class TransformListenerNode:
    def __init__(self, source_frame, target_frame):
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.listener = tf.TransformListener()
        
    def run(self):
        rate = rospy.Rate(10.0) #10Hz
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform(self.source_frame, self.target_frame, rospy.Time(0))
                rospy.loginfo("{} to {} Translation: {}".format(self.target_frame, self.source_frame, trans))
                rospy.loginfo("{} to {} Rotation: {}".format(self.target_frame, self.source_frame, rot))
                # print(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
        
def main():
    
    rospy.init_node('only read tf')
    source_frame = "base_link"
    target_frame = "tool0"
    tf_listener = TransformListenerNode(source_frame, target_frame)
    tf_listener.run()
    
    tag_size = 0.077  # Replace with your actual tag size
    detector = AprilTagDetector(tag_size)
    detector.run()
    
if __name__ == '__main__':
    main()
    