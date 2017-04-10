#! /usr/bin/env python
""" utility function for object detection
author: ren ye
changelog:
(2017-01-29) init
"""

import cv2
import numpy as np

# calibrated by palette
lower_red1 = np.array([0, 90, 90])
upper_red1 = np.array([25, 255, 255])
lower_red2 = np.array([175, 90, 90])
upper_red2 = np.array([255, 255, 255])
lower_blue = np.array([85, 90, 90])
upper_blue = np.array([130, 255, 255])
lower_green = np.array([25, 50, 75])
upper_green = np.array([75, 255, 255])


# void function for edge detector
def nothing(x):
    pass


# #### canny edge function ####
def canny_masking(hsv_mask, threshold1=100, threshold2=200, is_dynamic=False):
    """ take a mask and return an edge """
    if is_dynamic:  # with track bar
        # create trackbar on the window
        canny_hough_trackwindow(threshold1, threshold2)
        while (1):
            threshold1 = cv2.getTrackbarPos('Th_min', 'edge')
            threshold2 = cv2.getTrackbarPos('Th_max', 'edge')
            # find canny edge
            mask = cv2.Canny(hsv_mask, threshold1, threshold2,
                             L2gradient=True)
            # show the edge
            cv2.imshow("edge", mask)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
    else:  # with predefined thresholds
        mask = cv2.Canny(hsv_mask, threshold1, threshold2,
                         L2gradient=True)
    return mask, threshold1, threshold2


def hough_line_detection(img, mask, hsv_mask, thresholdHough=40,
                         minLineLength=10, maxLineGap=5, is_dynamic=False,
                         is_probabilistic=True):
    """ hough line detection """
    if is_probabilistic:  # probabilistic hough line
        if is_dynamic:  # with track bar
            # create track bar
            canny_hough_trackwindow(None, None,
                                    thresholdHough, minLineLength, maxLineGap)
            while (1):
                # mask hsv to original img
                hsv_masked_img = cv2.bitwise_and(img, img, mask=hsv_mask)
                # prob. hough lines
                thresholdHough = cv2.getTrackbarPos('Th_Hough', 'edge')
                minLineLength = cv2.getTrackbarPos('min_Linelength_Hough',
                                                   'edge')
                maxLineGap = cv2.getTrackbarPos('max_Linegap', 'edge')
                lines = cv2.HoughLinesP(mask.copy(), 1, np.pi / 180,
                                        thresholdHough, minLineLength,
                                        maxLineGap)
                try:  # in case no line is detected
                    print lines.shape
                    for l in lines:
                        for x1, y1, x2, y2 in l:
                            cv2.line(hsv_masked_img, (x1, y1), (x2, y2),
                                     (0, 255, 0), 2)
                except:
                    pass
                cv2.imshow("edge", hsv_masked_img)
                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break
        else:  # predefined thresholds
            hsv_masked_img = cv2.bitwise_and(img, img, mask=hsv_mask)
            lines = cv2.HoughLinesP(mask.copy(), 1, np.pi / 180,
                                    thresholdHough, minLineLength,
                                    maxLineGap)
            try:
                print lines.shape
                for l in lines:
                    for x1, y1, x2, y2 in l:
                        cv2.line(hsv_masked_img, (x1, y1), (x2, y2),
                                 (0, 255, 0), 2)
            except:
                pass
            cv2.imshow("edge", hsv_masked_img)
            cv2.waitKey(0)
    else:  # normal hough line detection
        if is_dynamic:  # with track bar
            canny_hough_trackwindow(None, None,
                                    thresholdHough, minLineLength, maxLineGap)
            while (1):
                hsv_masked_img = cv2.bitwise_and(img, img, mask=hsv_mask)
                lines = cv2.HoughLinesP(mask.copy(), 1, np.pi / 180,
                                        thresholdHough, minLineLength,
                                        maxLineGap)
                try:
                    print lines.shape
                    for l in lines:
                        for rho, theta in l:
                            a = np.cos(theta)
                            b = np.sin(theta)
                            x0 = a * rho
                            y0 = b * rho
                            x1 = int(x0 + 1000 * (-b))
                            y1 = int(y0 + 1000 * (a))
                            x2 = int(x0 - 1000 * (-b))
                            y2 = int(y0 - 1000 * (a))
                            cv2.line(hsv_masked_img, (x1, y1), (x2, y2),
                                     (0, 255, 0), 2)
                except:
                    pass
                cv2.imshow("edge", hsv_masked_img)
                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break
        else:  # predefined thresholds
            hsv_masked_img = cv2.bitwise_and(img, img, mask=hsv_mask)
            lines = cv2.HoughLinesP(mask.copy(), 1, np.pi / 180,
                                    thresholdHough, minLineLength,
                                    maxLineGap)
            try:
                print lines.shape
                for l in lines:
                    for rho, theta in l:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        cv2.line(hsv_masked_img, (x1, y1), (x2, y2),
                                 (0, 255, 0), 2)
            except:
                pass
            cv2.imshow("edge", hsv_masked_img)
            cv2.waitKey(0)


def canny_hough_trackwindow(threshold1=None, threshold2=None,
                            thresholdHough=None, minLineLength=None,
                            maxLineGap=None):
    """create track window for canny and hough"""
    # debugging use for user to dynamic change the thresholds
    if threshold1 is None:
        threshold1 = 100
    if threshold2 is None:
        threshold2 = 200
    if thresholdHough is None:
        thresholdHough = 40
    if minLineLength is None:
        minLineLength = 10
    if maxLineGap is None:
        maxLineGap = 5

    cv2.namedWindow("edge")
    cv2.createTrackbar('Th_min', 'edge', threshold1, 255, nothing)
    cv2.createTrackbar('Th_max', 'edge', threshold2, 255, nothing)
    cv2.createTrackbar('Th_Hough', 'edge', thresholdHough, 200, nothing)
    cv2.createTrackbar('min_Linelength_Hough', 'edge', minLineLength,
                       200, nothing)
    cv2.createTrackbar('max_Linegap', 'edge', maxLineGap, 100, nothing)


# #### hsv ####
def hsv_trackwindow(colorname="blue"):
    """ tracked window for hsv thresholding """

    # convert to hsv and do color filtering
    cv2.namedWindow("hsv")
    if colorname == 'blue':
        cv2.createTrackbar('Hl', 'hsv', lower_blue[0], 255, nothing)
        cv2.createTrackbar('Hu', 'hsv', upper_blue[0], 255, nothing)
        cv2.createTrackbar('Sl', 'hsv', lower_blue[1], 255, nothing)
        cv2.createTrackbar('Su', 'hsv', upper_blue[1], 255, nothing)
        cv2.createTrackbar('Vl', 'hsv', lower_blue[2], 255, nothing)
        cv2.createTrackbar('Vu', 'hsv', upper_blue[2], 255, nothing)
    elif colorname == 'red':
        cv2.createTrackbar('Hl', 'hsv', lower_red1[0], 255, nothing)
        cv2.createTrackbar('Hu', 'hsv', upper_red1[0], 255, nothing)
        cv2.createTrackbar('Sl', 'hsv', lower_red1[1], 255, nothing)
        cv2.createTrackbar('Su', 'hsv', upper_red1[1], 255, nothing)
        cv2.createTrackbar('Vl', 'hsv', lower_red1[2], 255, nothing)
        cv2.createTrackbar('Vu', 'hsv', upper_red1[2], 255, nothing)
    elif colorname == 'green':
        cv2.createTrackbar('Hl', 'hsv', lower_green[0], 255, nothing)
        cv2.createTrackbar('Hu', 'hsv', upper_green[0], 255, nothing)
        cv2.createTrackbar('Sl', 'hsv', lower_green[1], 255, nothing)
        cv2.createTrackbar('Su', 'hsv', upper_green[1], 255, nothing)
        cv2.createTrackbar('Vl', 'hsv', lower_green[2], 255, nothing)
        cv2.createTrackbar('Vu', 'hsv', upper_green[2], 255, nothing)


def hsv_masking(img, colorname="blue", is_dynamic=False):
    """ masking color image by color """

    # convert from BGR to HSV
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if is_dynamic:  # with trackbar
        hsv_trackwindow(colorname)
        while (1):
            hsv_mask = cv2.inRange(img_hsv,
                                   np.array([cv2.getTrackbarPos('Hl', 'hsv'),
                                             cv2.getTrackbarPos('Sl', 'hsv'),
                                             cv2.getTrackbarPos('Vl', 'hsv')]),
                                   np.array([cv2.getTrackbarPos('Hu', 'hsv'),
                                             cv2.getTrackbarPos('Su', 'hsv'),
                                             cv2.getTrackbarPos('Vu', 'hsv')]))

            cv2.imshow("hsv", hsv_mask)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
    else:  # predefined color
        if colorname == "blue":
            hsv_mask = cv2.inRange(img_hsv, lower_blue, upper_blue)
        elif colorname == "green":
            hsv_mask = cv2.inRange(img_hsv, lower_green, upper_green)
        elif colorname == "red":
            hsv_mask = cv2.inRange(img_hsv, lower_red1, upper_red1) + \
                cv2.inRange(img_hsv, lower_red2, upper_red2)
        cv2.imshow("hsv", hsv_mask)
    return hsv_mask


# #### contour detector ####
def find_contour(img, mask, is_max=True):
    """ find contours and moments for the edge """
    # find all contours
    hierarchy, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)

    # save the moments
    features = {"area": list(),
                "arclength": list(),
                "M": list(),
                "mass_center": list(),
                "aspect_ratio": list(),
                "rect_area": list(),
                "extent": list(),
                "circle_center": list(),
                "solidity": list()}
    for i, cnt in enumerate(contours):
        print 'no: ', i
        # area and perimeter
        features["area"].append(cv2.contourArea(cnt))
        print 'area: ', features["area"][i]
        features["arclength"].append(cv2.arcLength(cnt, True))
        print 'arc length: ', features["arclength"][i]
        # centroid
        features["M"].append(cv2.moments(cnt))
        centroid_x = int(features["M"][i]['m10'] / features["M"][i]['m00'])
        centroid_y = int(features["M"][i]['m01'] / features["M"][i]['m00'])
        features["mass_center"].append((centroid_x, centroid_y))
        print 'centroid: ', features["mass_center"][i]
        # draw the mass center
        cv2.circle(img, features["mass_center"][i], 1, (0, 0, 255), -1)
        # aspect ratio
        rect = cv2.minAreaRect(cnt)
        (x, y), (w, h), angle = rect
        features["aspect_ratio"].append(float(w) / h)
        print 'aspect ratio: ', features["aspect_ratio"][i]
        # extent
        features["rect_area"].append(w * h)
        features["extent"].append(features["area"][i] /
                                  features["rect_area"][i])
        print "extent: ", features["extent"]
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # draw min area rect
        # cv2.drawContours(img, [box], 0, (0, 0, 255), 1)
        # mimimum enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        features["circle_center"].append(center)
        radius = int(radius)
        print 'circle_center: ', features["circle_center"][i]
        # draw min enclosing circle
        cv2.circle(img, center, 1, (255, 0, 255), -1)
        # cv2.circle(img, center, radius, (255, 0, 255), 1)
        # solidity
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        hull_m = cv2.moments(hull)
        hull_x = int(hull_m['m10'] / hull_m['m00'])
        hull_y = int(hull_m['m01'] / hull_m['m00'])
        features["solidity"].append(float(features["area"][i]) / hull_area)
        print 'hull center: ', (hull_x, hull_y)
        print 'solidity: ', features["solidity"][i]
        cv2.drawContours(img, [hull], 0, [0, 255, 0], 1)
        cv2.circle(img, (hull_x, hull_y), 1, (0, 255, 0), -1)

        # orientation
        ellipse = cv2.fitEllipse(cnt)
        (x, y), (MA, ma), angle = ellipse
        print 'orientation: ', angle
        print 'ellipse center: ', (x, y)
        # draw boundign ellipse
        cv2.ellipse(img, ellipse, (0, 0, 255), 1)
        cv2.circle(img, (int(x), int(y)), 1, (0, 0, 255), -1)

        cv2.imshow("image", img)
    if is_max:
        return features, np.argmax(features["area"])
    else:
        return features, len(features)


def find_good_keypoint(img, mask,
                       maxCorners=10, qualityLevel=0.01, minDistance=5,
                       is_dynamic=False):
    """ find keypoint using good features """
    if is_dynamic:
        kpt_trackwindow(maxCorners, int(qualityLevel * 100), minDistance)
        condition = True
        while (condition):
            maxc = cv2.getTrackbarPos("maxC", "kpt")
            mind = cv2.getTrackbarPos("minD", "kpt")
            qlvl = cv2.getTrackbarPos("qLvl", "kpt") / 100.0
            corners = cv2.goodFeaturesToTrack(mask, maxc, qlvl, mind)
            corners = np.int0(corners)
            for i in corners:
                x, y = i.ravel()
                cv2.circle(img, (x, y), 3, (0, 255, 255), 1)

            cv2.imshow("kpt", img)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                condition = False
        else:
            gd_x = sum([i.ravel()[0] for i in corners]) / len(corners)
            gd_y = sum([i.ravel()[1] for i in corners]) / len(corners)
    else:
        maxc = maxCorners
        mind = qualityLevel
        qlvl = minDistance
        corners = cv2.goodFeaturesToTrack(mask, maxc, qlvl, mind)
        corners = np.int0(corners)
        gd_x = sum([i.ravel()[0] for i in corners]) / len(corners)
        gd_y = sum([i.ravel()[1] for i in corners]) / len(corners)
        print "good center: ", (gd_x, gd_y)
        cv2.circle(img, (gd_x, gd_y), 2, (0, 255, 255), -1)

        for i in corners:
            x, y = i.ravel()
            cv2.circle(img, (x, y), 3, (0, 255, 255), 1)

        cv2.imshow("kpt", img)
    return (gd_x, gd_y), corners


def find_orb_keypoint(img, mask):

    # orb keypoint detection
    orb = cv2.ORB_create(edgeThreshold=25, patchSize=31, nlevels=8,
                         fastThreshold=20, scaleFactor=1.2,
                         WTA_K=2, scoreType=cv2.ORB_FAST_SCORE,
                         firstLevel=0, nfeatures=500)
    kp = orb.detect(mask)
    kp_x = sum([i.pt[0] for i in kp]) / len(kp)
    kp_y = sum([i.pt[1] for i in kp]) / len(kp)

    img2 = cv2.drawKeypoints(img, kp, None, color=(0, 255, 0), flags=0)
    cv2.imshow("kpt", img2)
    return (kp_x, kp_y)


def kpt_trackwindow(maxCorners=10, qualityLevel=1, minDistance=5):
    """ tracked window for hsv thresholding """

    # convert to hsv and do color filtering
    cv2.namedWindow("kpt")
    cv2.createTrackbar('maxC', 'kpt', maxCorners, 15, nothing)
    cv2.createTrackbar('qLvl', 'kpt', qualityLevel, 100, nothing)
    cv2.createTrackbar('minD', 'kpt', minDistance, 50, nothing)


def cal_direction(circle_center, keypoint_center):
    """ use keypoint center and min enclosing circle center
    to calculate the direction """
    cx, cy = circle_center
    kx, ky = keypoint_center

    return np.arctan2((ky - cy), (kx - cx))
