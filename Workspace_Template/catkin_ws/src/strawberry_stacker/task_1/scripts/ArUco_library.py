#!/usr/bin/env python3
'''
This module is a library for ArUco detection in OpenCV.
'''
import numpy
import math
import cv2
from cv2 import aruco

def detect_ArUco(img):
    '''
    Detecting the Arucos in the image and extracting ID and coordinate values.
    '''
    Detected_ArUco_markers = {}
    corners = []
    ids = []
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

    # verify *at least* one ArUco marker was detected
    if len(corners)>0 :
            # flatten the ArUco IDs list
        ids = ids.flatten()
            # loop over the detected ArUCo corners
        for (marker_corner, marker_id) in zip(corners, ids):
            Detected_ArUco_markers[marker_id] = marker_corner

    return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
    '''
    Calculates the orientation of each detected ArUco.
    '''
    ArUco_marker_angles = {}
    for key, _ in Detected_ArUco_markers.items():
        x_0, y_0 = map(int, Detected_ArUco_markers[key][0][0])
        x_1, y_1 = map(int, Detected_ArUco_markers[key][0][1])
        x_2, y_2 = map(int, Detected_ArUco_markers[key][0][2])
        #calculating midpoint between topleft and topright
        mpx, mpy = map(int, (((x_0+x_1)/2), ((y_0+y_1)/2)))
        c_x = int((x_0+x_2)/2)
        c_y = int((y_0+y_2)/2 )
        angle = int(math.degrees(math.atan2(mpy-c_y,mpx-c_x)))


        if angle>0:
            angle = 360-angle
        else:
            angle = abs(angle)
        ArUco_marker_angles[key] = angle
    #returning the angles of the ArUco markers in degrees as a dictionary
    return ArUco_marker_angles


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
    '''
    Creates masks over the ArUcos with angle, ID and other data.
    '''
    ## enter your code here ##
    for aruco_id,angle in ArUco_marker_angles.items():
        x_0, y_0 = map(int, Detected_ArUco_markers[aruco_id][0][0])
        x_1, y_1 = map(int, Detected_ArUco_markers[aruco_id][0][1])
        x_2, y_2 = map(int, Detected_ArUco_markers[aruco_id][0][2])
        x_3, y_3 = map(int, Detected_ArUco_markers[aruco_id][0][3])
        #calculating midpoint between topleft and topright
        mpx, mpy = map(int,(((x_0+x_1)/2), ((y_0+y_1)/2)))
        c_x = int((x_0+x_2)/2)
        c_y = int((y_0+y_2)/2 )
        cv2.circle(img, (x_0,y_0) , 5, (125,125,125), -1)   #gray dot
        cv2.circle(img, (x_1,y_1) , 5, (0,255,0), -1)       #green dot
        cv2.circle(img, (x_2,y_2) , 5, (180,105,255), -1)   #pink dot
        cv2.circle(img, (x_3,y_3) , 5, (255,255,255), -1)   #white dot
        cv2.circle(img, (c_x,c_y) , 5, (0,0,255), -1)       #red dot at the centre
        cv2.line(img, (c_x,c_y), (mpx,mpy), (255,0,0), 5)   #blue line connecting centre and midpt

        #printing ID on the right of centre
        cv2.putText(img, str(aruco_id), (c_x+50,c_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        #printing orientation on left of centre
        cv2.putText(img, str(angle), (c_x-100,c_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        #printing the team ID
        cv2.putText(img, "SS_1302", (40,40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 2)
    return img
