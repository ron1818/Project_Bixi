#! /usr/bin/env python
""" detect arrow and direction printed on the side box
author: ren ye
changelog:
(2017-01-29) init
"""

import cv2
# import numpy as np
from cv_utils import *


# ## main ##
# picture folder
image_path = "image/arrow.png"
# image_path = "image/blue_right.png"

# #### load picture ####
img = cv2.imread(image_path, 1)
rotated_angle = 45
# rotate image
rows, cols, _ = img.shape
M = cv2.getRotationMatrix2D((cols / 2, rows / 2), rotated_angle, 1)
dst = cv2.warpAffine(img, M, (cols, rows))
cv2.imshow("rotated", dst)
cv2.waitKey(0)

threshold1 = 100
threshold2 = 200
thresholdHough = 40
minLineLength = 6
maxLineGap = 5

hsv_mask = hsv_masking(dst, is_dynamic=True)
# mask, _, _ = canny_masking(hsv_mask, is_dynamic=True)
# hough_line_detection(img, mask, hsv_mask,
#                      is_dynamic=True, is_probabilistic=True)

contour_param, max_idx = find_contour(dst, hsv_mask, is_max=True)

# ret, hsv_mask_inv = cv2.threshold(hsv_mask, 127, 255, cv2.THRESH_BINARY_INV)
kpt = find_keypoint(hsv_mask, maxCorners=7)
# print kpt

# orb keypoint detection
orb = cv2.ORB_create(edgeThreshold=25, patchSize=31, nlevels=8,
                     fastThreshold=20, scaleFactor=1.2,
                     WTA_K=2, scoreType=cv2.ORB_FAST_SCORE,
                     firstLevel=0, nfeatures=500)
kp = orb.detect(hsv_mask)
kp_x = sum([kp[i].pt[0] for i in range(len(kp))]) / len(kp)
kp_y = sum([kp[i].pt[1] for i in range(len(kp))]) / len(kp)
print "keypoint center: ", (kp_x, kp_y)

circle_center = contour_param['circle_center'][max_idx]
print np.degrees(cal_direction(circle_center, (kp_x, kp_y)))
img2 = cv2.drawKeypoints(dst, kp, None, color=(0, 255, 0), flags=0)
cv2.imshow("kpts", img2)
cv2.waitKey(0)
