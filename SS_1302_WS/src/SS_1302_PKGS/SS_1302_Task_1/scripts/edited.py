#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import SS_1302_aruco_library as arucoLib

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker
		


	# Callback function of amera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return
			
	def publish_data(self):
		self.marker_pub.publish(self.marker_msg)

	def Centre_pos(self):
		x_0, y_0 = map(int, Detected_ArUco_markers[key][0][0])
		x_1, y_1 = map(int, Detected_ArUco_markers[key][0][1])
		x_2, y_2 = map(int, Detected_ArUco_markers[key][0][2])
        #calculating midpoint between topleft and topright
		mpx, mpy = map(int, (((x_0+x_1)/2), ((y_0+y_1)/2)))
		c_x = int((x_0+x_2)/2)
		c_y = int((y_0+y_2)/2 )
		return c_x,c_y

if __name__ == '__main__':
	image_proc_obj = image_proc()
	image_proc_obj.Detected_markers = arucoLib.detect_ArUco(image_proc_obj.img)
	image_proc_obj.orientation = arucoLib.Calculate_orientation_in_degree(image_proc_obj.Detected_markers)

	marker_msg_obj = Marker()
	marker_msg_obj.x = 10
	marker_msg_obj.y = 12

	rospy.spin()
