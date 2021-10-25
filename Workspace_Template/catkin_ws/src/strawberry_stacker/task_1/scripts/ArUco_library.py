#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}

    Detected_ArUco_markers = {}
    ## enter your code here ##
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    print(ids)                                                                                #DELETEEEEEEEE
    # verify *at least* one ArUco marker was detected
    if len(corners)>0 :
			# flatten the ArUco IDs list
      ids = ids.flatten()
			# loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):
      Detected_ArUco_markers[markerID] = markerCorner
   
    return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	## enter your code here ##
	c = 0
	for key,coordinates in Detected_ArUco_markers.items():
		global x0,x1,x2,x3,y0,y1,y2,y3,mpx,mpy,cx,cy
		x0, y0 = map(int, coordinates[c][0])
		x1, y1 = map(int, coordinates[c][1])
		x2, y2 = map(int, coordinates[c][2])
		x3, y3 = map(int, coordinates[c][3])
		mpx, mpy = map(int,(((x0+x1)/2) , ((y0+y1)/2 )) )    #calculating midpoint between topleft and topright
		cx = int((x0+x2)/2)
		cy = int((y0+y2)/2 )
		angle = int(math.degrees(math.atan2(mpy-cy,mpx-cx)))


		if angle<0:
			angle = 360-angle
		ArUco_marker_angles[key] = angle

	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##
    for arucoID,angle in ArUco_marker_angles.items():
      cv2.circle(img, (x0,y0) , 5, (125,125,125), -1)              #gray dot
      cv2.circle(img, (x1,y1) , 5, (0,255,0), -1)                  #green dot
      cv2.circle(img, (x2,y2) , 5, (180,105,255), -1)              #pink dot
      cv2.circle(img, (x3,y3) , 5, (255,255,255), -1)              #white dot
      cv2.circle(img, (cx,cy) , 5, (0,0,255), -1)                  #red dot at the centre
      cv2.line(img, (cx,cy), (mpx,mpy), (255,0,0), 5)              #blue line connecting the centre and the midpoint
		
      cv2.putText(img, str(arucoID), (cx+100,cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 1)          #printing ID on the right of centre
      cv2.putText(img, str(angle), (cx-100,cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1)            #printing orientation on left of centre
    return img


