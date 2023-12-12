#!/usr/bin/python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import math
import numpy as np


import cv2
import numpy as np
from matplotlib import pyplot as plt

def remove_grabcut_bg(image):
    tmp = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
    _,alpha = cv2.threshold(tmp,0,255,cv2.THRESH_BINARY)
    r, g, b = cv2.split(image)
    rgba = [r,g,b,alpha]
    dst = cv2.merge(rgba,4)
    return dst

size = (720, 720)
src = cv2.imread("/home/irol/Downloads/20230207_151847.jpg", cv2.IMREAD_COLOR)
src = cv2.resize(src, dsize=(720, 720))

dst = src.copy()
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
canny = cv2.Canny(gray, 5000, 1500, apertureSize = 5, L2gradient = True)
lines = cv2.HoughLines(canny, 0.8, np.pi / 180, 150, srn = 100, stn = 200, min_theta = np.pi/2, max_theta = np.pi)

for i in lines:
    rho, theta = i[0][0], i[0][1]
    a, b = np.cos(theta), np.sin(theta)
    x0, y0 = a*rho, b*rho

    scale = src.shape[0] + src.shape[1]

    x1 = int(x0 + scale * -b)
    y1 = int(y0 + scale * a)
    x2 = int(x0 - scale * -b)
    y2 = int(y0 - scale * a)

    cv2.line(dst, (x1, y1), (x2, y2), (0, 0, 255), 2)
    
cv2.imshow("before", src)
cv2.imshow("after", dst)
cv2.waitKey(0)
cv2.destroyAllWindows()
# cv2.imshow('ori', src)

# output = src[420:460, 120:600]
# gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

# sobel = cv2.Sobel(gray, cv2.CV_8U, 1, 0, 3)
# laplacian = cv2.Laplacian(gray, cv2.CV_8U, ksize=3)
# canny = cv2.Canny(src, 100, 255)

# laplacian_rgb = cv2.cvtColor(laplacian, cv2.COLOR_GRAY2BGR)
# canny_rgb = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)

# laplacian_rgb_ = remove_grabcut_bg(canny_rgb)
# laplacian_rgb_ = cv2.cvtColor(laplacian_rgb_, cv2.COLOR_BGRA2BGR)##

# bgr = laplacian_rgb_[:,:,0:3]

# # select grayscale range
# mask = cv2.inRange(bgr, (210,210,210), (255,255,255))

# # change the image to make it green where the mask is white
# bgr_new = bgr.copy()

# # BGR이라서 255, 0, 0
# bgr_new[mask!=255] = (255,0,0)

# # save output
# cv2.imshow('bgr_new.png', bgr_new)

# print(laplacian_rgb_.shape)

# # src[420:460, 120:600] = laplacian_rgb_

# # cv2.imshow('result',src)
# # cv2.waitKeyEx()
# # cv2.destroyAllWindows()

# print(laplacian_rgb_.shape)
# img = cv2.addWeighted(src, 0.6, laplacian_rgb_ , 1, 0)

# cv2.imshow('laplacian', laplacian)
# cv2.imshow('laplacian_rgb', laplacian_rgb)
# cv2.imshow('canny', canny)
# cv2.imshow('img', img)
# # cv2.imshow('laplacian_rgb_',laplacian_rgb_)
# # cv2.imshow('d', remove_grabcut_bg(laplacian_rgb_))

# cv2.waitKeyEx()
# cv2.destroyAllWindows()


# size = (720, 720)
# src = cv2.imread("/home/irol/Downloads/20230207_151847.jpg", cv2.IMREAD_COLOR)
# src = cv2.resize(src, dsize=(720, 720))

# cv2.imshow('ori', src)

# output = src[420:460, 120:600]
# gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

# sobel = cv2.Sobel(gray, cv2.CV_8U, 1, 0, 3)
# laplacian = cv2.Laplacian(gray, cv2.CV_8U, ksize=3)
# canny = cv2.Canny(src, 100, 255)

# laplacian_rgb = cv2.cvtColor(laplacian, cv2.COLOR_GRAY2BGR)
# canny_rgb = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)

# img2gray = cv2.cvtColor(laplacian_rgb, cv2.COLOR_BGR2GRAY)
# ret, mask = cv2.threshold(img2gray, 250, 255, cv2.THRESH_BINARY)
# mask_inv = cv2.bitwise_not(mask)

# cv2.imshow('mask',mask)
# cv2.imshow('mask_inv',mask_inv)


# mask_inv = cv2.cvtColor(mask_inv, cv2.COLOR_GRAY2BGR)
# img_mask = cv2.cvtColor(mask_inv, cv2.COLOR_BGR2GRAY)

# src[420:460, 120:600] = mask_inv

# cv2.imshow('result',src)
# cv2.waitKeyEx()
# cv2.destroyAllWindows()

