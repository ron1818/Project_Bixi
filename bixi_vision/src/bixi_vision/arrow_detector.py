#! /usr/bin/env python
""" detect arrow and direction printed on the side box
author: ren ye
changelog:
(2017-01-29) init
"""

import cv2
import numpy as np

# picture folder
image_path = "image/arrow.png"

# load picture
img = cv2.imread(image_path, 1)
# cv2.imshow("orig", img)
# cv2.waitKey(0)

# convert to gray
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# cv2.imshow("gray", img_gray)
# cv2.waitKey(0)

# canny edge
threshold1 = 100
threshold2 = 200
gray = cv2.equalizeHist(img_gray)
mask = cv2.Canny(gray, threshold1, threshold2, L2gradient=True)
cv2.imshow("edge", mask)
cv2.waitKey(0)

