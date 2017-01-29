#! /usr/bin/env python
""" detect arrow and direction printed on the side box
author: ren ye
changelog:
(2017-01-29) init
"""

import cv2
import numpy as np
from cv_utils import *


## main ##
# picture folder
image_path = "image/blue_right.png"

#### load picture ####
img = cv2.imread(image_path, 1)
threshold1 = 100
threshold2 = 200
thresholdHough = 40
minLineLength = 6
maxLineGap = 5

hsv_mask = hsv_masking(img, is_dynamic=True)
mask, _, _ = canny_masking(hsv_mask, is_dynamic=True)
hough_line_detection(img, mask, hsv_mask,
                     is_dynamic=True, is_probabilistic=True)
