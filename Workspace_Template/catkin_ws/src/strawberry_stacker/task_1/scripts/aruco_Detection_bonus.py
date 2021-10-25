#!/usr/bin/env python3

############## Task1.1 - ArUco Detection ##############
### YOU CAN EDIT THIS FILE FOR DEBUGGING PURPOSEs, SO THAT YOU CAN TEST YOUR ArUco_library.py AGAINST THE VIDEO Undetected ArUco markers.avi###
### BUT MAKE SURE THAT YOU UNDO ALL THE CHANGES YOU HAVE MADE FOR DEBUGGING PURPOSES BEFORE TESTING AGAINST THE TEST IMAGES ###

import numpy as np
import cv2
import cv2.aruco as aruco
import time
from ArUco_library import *

#cap = cv2.VideoCapture('/home/apex_18/problem_vid.mp4')
cap = cv2.VideoCapture(0)
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
   
size = (frame_width, frame_height)

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

# Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 20, (frame_width,frame_height))

while(True):
  ret, img = cap.read()


  if ret == True: 
    Detected_ArUco_markers = detect_ArUco(img)									## detecting ArUco ids and returning ArUco dictionary
    angle = Calculate_orientation_in_degree(Detected_ArUco_markers)				## finding orientation of aruco with respective to the menitoned scale in problem statementimg = mark_ArUco(img,Detected_ArUco_markers,angle)						## marking the parameters of aruco which are mentioned in the problem statement
    final = mark_ArUco(img,Detected_ArUco_markers,angle)
    
    # Write the frame into the file 'output.avi'
    out.write(final)

    # Display the resulting frame    
    cv2.imshow('frame',frame)

    # Press Q on keyboard to stop recording
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  # Break the loop
  else:
    break  

# When everything done, release the video capture and video write objects
cap.release()
out.release()

# Closes all the frames
cv2.destroyAllWindows()



