#!/usr/bin/env python3

import rospy
import cv2
import apriltag
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class AprilTagDetector:
    def __init__(self):
        # ROS setup
        self.bridge = CvBridge()
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.detector = apriltag.Detector()

    def color_callback(self, data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray_image)
            for r in results:
                self.visualize_tag(r, color_image)
                
            cv2.imshow("AprilTag Detector", color_image)
            cv2.waitKey(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                
        except CvBridgeError as e:
            rospy.logerr("Could not convert color image. Error: %s", str(e))

    def visualize_tag(self, r, color_image):
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

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('apriltag_detector')
    detector = AprilTagDetector()
    
    try:
        detector.run()
    except rospy.ROSInterruptException:
        pass
    
if __name__ == '__main__':
    main()