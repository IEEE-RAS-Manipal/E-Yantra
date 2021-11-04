#!/usr/bin/env python3
'''
This module conducts marker detection of a moving ArUco marker.
'''

import math
from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import Image
from ss_1302_task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from cv2 import aruco
import numpy as np
import rospy


class ImageProc():
    '''
    This class contains functions for ArUco marker processing.
    '''

    # Initialise everything
    def __init__(self):
        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.detected_aruco_markers = {}

        # Initialise rosnode
        rospy.init_node('marker_detection', anonymous=True)
        self.rate = rospy.Rate(10)
        # This will contain the message structure of message type ss_1302_task_1/Marker
        self.marker_msg = Marker()

        # Making a publisher
        self.marker_pub = rospy.Publisher(
            '/marker_info', Marker, queue_size=1)
        # Subscribing to the camera topic /camera/camera/image_raw
        self.image_sub = rospy.Subscriber(
            "/camera/camera/image_raw", Image, self.image_callback)

    def detect_aruco(self, img):
        '''
        Detecting the Arucos in the image and extracting ID and coordinate values.
        '''
        corners = []
        ids = []
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            img, aruco_dict, parameters=parameters)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (marker_corner, marker_id) in zip(corners, ids):
                self.detected_aruco_markers[marker_id] = marker_corner

    def image_callback(self, data):
        '''
        Callback function of the camera topic.
        '''
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            print(err)
            return

    def publish_data(self):
        '''
        Publishes the marker data.
        '''
        self.marker_pub.publish(self.marker_msg)

    def marker_pos(self):
        '''
        Calculates the position and orientation of the marker (centre position).
        '''
        for key, coods in self.detected_aruco_markers.items():
            # Calculating relevant point coordinates
            x_0, y_0 = map(int, coods[0][0])
            x_1, y_1 = map(int, coods[0][1])
            x_2, y_2 = map(int, coods[0][2])

            # Calculating centre position
            c_x = int((x_0+x_2)/2)
            c_y = int((y_0+y_2)/2)

            # Calculating orientation angle
            mpx, mpy = map(int, (((x_0+x_1)/2), ((y_0+y_1)/2)))
            angle = int(math.degrees(math.atan2(c_y-mpy, c_x-mpx)))
            if angle < 0:
                angle = 360+angle

            # Filling in message data
            self.marker_msg.x = c_x
            self.marker_msg.y = c_y
            self.marker_msg.z = int(key)
            self.marker_msg.yaw = angle
            self.publish_data()  # Publishing data


if __name__ == '__main__':
    image_proc_obj = ImageProc()  # Creating object
    image_proc_obj.rate.sleep()  # Delay for syncing
    try:
        while not rospy.is_shutdown():  # Will run till node active
            image_proc_obj.rate.sleep()  # Delay
            # Detecting ArUco marker
            image_proc_obj.detect_aruco(image_proc_obj.img)
            # Calculating and publishing position/orientation data
            image_proc_obj.marker_pos()
    except ROSInterruptException:
        pass

    rospy.spin()
