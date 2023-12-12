#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math as m
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Float32MultiArray

def get_pipeline():
    # Create a pipeline
    pipeline = rs.pipeline()


    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # x: 0 ~ 169.33mm / y: 0 ~ 127mm
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) # x: 0 ~ 338.67mm / y: 0 ~ 190.5mm
    # config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    clipping_distance_in_meters = 1 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    align_to = rs.stream.color #align depth to color
    align = rs.align(align_to) #generate align for align depth to color

    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    print(depth_intrinsics)
    w, h = depth_intrinsics.width, depth_intrinsics.height

    return pipeline, align, clipping_distance, depth_intrinsics

def get_frame(pipeline, align, clipping_distance, depth_intrinsics):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame() # aligned depth frame
    color_frame = aligned_frames.get_color_frame() # aligned color frame

    # Validate that both frames are valid
    if not depth_frame or not color_frame:
        return None, None, None, None

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Remove background - Set pixels further than clipping_distance to grey
    grey_color = 153
    depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    # Render images
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    images = np.hstack((bg_removed, depth_colormap))
    # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('Align Example', images)
    cv2.waitKey(1)

    return depth_image, color_image, bg_removed, depth_intrinsics, images, depth_colormap

def get_depth_point(depth_image, depth_intrinsics, x, y):
    depth = depth_image[y][x]
    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)
    return depth_point

def get_distance(depth_point):
    return m.sqrt(depth_point[0]**2 + depth_point[1]**2 + depth_point[2]**2)

def get_angle(depth_point):
    return m.atan2(depth_point[1], depth_point[0])


X = 0
Y = 0
Z = 0
count = 0
def mouse_callback(event, x, y, flags, param):
    global X, Y, Z, count
    if event == cv2.EVENT_LBUTTONDOWN:
        depth_point = get_depth_point(depth_image, depth_intrinsics, x, y)
        distance = get_distance(depth_point)
        angle = get_angle(depth_point)
        pixel_x = x
        pixel_y = y

                # 680x480
            # if origin is at the 'leftdown' of the image
        # camera_x = x*0.264583333
        # camera_y = (480-y)*0.264583333
        # print('--------------------------------------------')
        # print('pixel_x: ', pixel_x, 'pixel_y: ', pixel_y)
        # print('camera_x: ', round(camera_x, 3), 'mm, ', 'camera_y: ', round(camera_y, 3), 'mm')
        # print('distance: ', round(distance, 3), 'mm, ', 'angle: ', round(angle, 3), 'rad')
        # print('--------------------------------------------')
            # if origin is at the 'centerdown' of the image
        # camera_x = (x-320)*0.264583333
        # camera_y = (480-y)*0.264583333
        # print('--------------------------------------------')
        # print('pixel_x: ', pixel_x, 'pixel_y: ', pixel_y)
        # print('camera_x: ', round(camera_x, 3), 'mm, ', 'camera_y: ', round(camera_y, 3), 'mm')
        # print('distance: ', round(distance, 3), 'mm, ', 'angle: ', round(angle, 3), 'rad')
        # print('--------------------------------------------')

                # 1280x720
            # if origin is at the 'leftdown' of the image
        # camera_x = x*0.264583333
        # camera_y = (720-y)*0.264583333
        # print('-------------------leftdown-------------------')
        # print('pixel_x: ', pixel_x, 'pixel_y: ', pixel_y)
        # print('camera_x: ', round(camera_x, 3), 'mm, ', 'camera_y: ', round(camera_y, 3), 'mm')
        # print('distance: ', round(distance, 3), 'mm, ', 'angle: ', round(angle, 3), 'rad')
        # print('--------------------------------------------')
            # if origin is at the 'centerdown' of the image
        # camera_x = (x-640)*0.264583333
        # camera_y = (720-y)*0.264583333
        # print('-------------------------------centerdown-------------------------------')
        # print(f"pixel_x: {pixel_x}, pixel_y: {pixel_y}")
        # print(f"camera_x: {round(camera_x, 3)} mm, camera_y: {round(camera_y, 3)} mm")
        # print(f"distance: {round(distance, 3)} mm, angle: {round(angle, 3)} rad")
        # print('------------------------------------------------------------------------')
            # if origin is at the 'center' of the image
        # camera_x = (x-640)*0.264583333
        # camera_y = (360-y)*0.264583333
        # print('----------------------------------center----------------------------------')
        # print(f"pixel_x: {pixel_x}, pixel_y: {pixel_y}")
        # print(f"camera_x: {round(camera_x, 3)} mm, camera_y: {round(camera_y, 3)} mm")
        # print(f"distance: {round(distance, 3)} mm, angle: {round(angle, 3)} rad")
        # print('------------------------------------------------------------------------')

        cx, cy, fx, fy  = 317.858, 241.466, 383.553, 383.553
        # cx, cy, fx, fy  = 636.43, 362.423 ,639.255, 639.255

        u = ((pixel_x) - cx) / fx
        v = ((pixel_y) - cy) / fy

        X = u * distance
        Y = -v * distance
        a = distance
        b = m.sqrt(X**2 + Y**2)
        Z = m.sqrt(a**2 - b**2)
        count = count + 1
        # print('----------------------------------face----------------------------------')
        print(f"X: {round(X, 3)} mm, Y: {round(Y, 3)} mm, Z: {round(Z, 3)} mm, distance: {round(distance, 3)} mm")
        print('------------------------------------------------------------------------')


if __name__ == '__main__':
    pipeline, align, clipping_distance, depth_intrinsics = get_pipeline()

    rospy.init_node('camera_calibration', anonymous=True)
    publisher = rospy.Publisher('/R_coordinate', Float32MultiArray, queue_size=10)

    try:
        while True:
            depth_image, color_image, bg_removed, depth_intrinsics, images, depth_colormap = get_frame(pipeline, align, clipping_distance, depth_intrinsics)
            if depth_image is None:
                continue

            # 680x480
            color_image = cv2.line(color_image, (320, 460), (320, 480), (0, 255, 0), 5)
            color_image = cv2.line(color_image, (320, 480), (340, 480), (0, 0, 255), 5)
            cv2.circle(color_image, (320, 240), 5, (255, 0, 0), -1)
            cv2.imshow('RealSense', color_image)

            # 1280x720
            # color_image = cv2.line(color_image, (640, 700), (640, 720), (0, 255, 0), 5)
            # color_image = cv2.line(color_image, (640, 720), (660, 720), (0, 0, 255), 5)
            # cv2.circle(color_image, (640, 360), 5, (255, 0, 0), -1)
            # cv2.imshow('RealSense', color_image)
            
            cv2.setMouseCallback('RealSense', mouse_callback)
            
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

            if count == 1:
                print(X, Y, Z, count)
                count = 0
                
                msg = Float32MultiArray()
                msg.data = [X, Y, Z]
                
                publisher.publish(msg)
                
                
            time.sleep(0.1)

    finally:
        pipeline.stop()
    