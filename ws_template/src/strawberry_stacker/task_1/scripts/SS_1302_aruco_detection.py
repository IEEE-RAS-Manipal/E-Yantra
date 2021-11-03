#!/usr/bin/env python3
'''
This module is the detection.py file for the image aruco detection submission.
'''

import cv2
from cv2 import aruco
from SS_1302_aruco_library import *


image_list = ["../scripts/test_image1.png", "../scripts/test_image2.png"]
TEST_NUM = 1

for image in image_list:
    img = cv2.imread(image)
    # detecting ArUco ids and returning ArUco dictionary
    Detected_ArUco_markers = detect_ArUco(img)
    # finding orientation of aruco with respective to the menitoned scale in problem statement
    angle = Calculate_orientation_in_degree(Detected_ArUco_markers)
    # marking the parameters of aruco which are mentioned in the problem statement
    img = mark_ArUco(img, Detected_ArUco_markers, angle)
    RESULT_IMAGE = "../scripts/Result_image" + str(TEST_NUM)+".png"
    cv2.imwrite(RESULT_IMAGE, img)  # saving the result image
    TEST_NUM = TEST_NUM + 1
