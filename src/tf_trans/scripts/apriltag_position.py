import pyrealsense2 as rs
import numpy as np
import cv2
import apriltag

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start the camera pipeline
pipeline.start(config)

# Create an AprilTag detector object
detector = apriltag.Detector()

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
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
        detections = detector.detect(gray_image)

        # Loop over the detections
        for detection in detections:
            # Extract the bounding box (x, y)-coordinates
            (ptA, ptB, ptC, ptD) = detection.corners
            ptA = (int(ptA[0]), int(ptA[1]))
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))

            # Draw the bounding box of the AprilTag detection
            # cv2.line(color_image, ptA, ptB, (0, 255, 0), 2)
            # cv2.line(color_image, ptB, ptC, (0, 255, 0), 2)
            # cv2.line(color_image, ptC, ptD, (0, 255, 0), 2)
            # cv2.line(color_image, ptD, ptA, (0, 255, 0), 2)

            # Draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(detection.center[0]), int(detection.center[1]))
            cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)

            # Draw the tag ID on the image
            cv2.putText(color_image, str(detection.tag_id), (ptA[0], ptA[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Get the depth at the center of the AprilTag
            depth = depth_frame.get_distance(cX, cY)
            
            # Convert depth to a more human-readable format (e.g., meters or centimeters)
            depth_in_meters = depth  # The depth is already in meters

            print(f"ID: {detection.tag_id}, Center: {cX, cY}, Depth: {round(depth_in_meters, 3)}m")

        # Display the color image
        cv2.imshow('RealSense', color_image)
        
        # Breaking out of the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
