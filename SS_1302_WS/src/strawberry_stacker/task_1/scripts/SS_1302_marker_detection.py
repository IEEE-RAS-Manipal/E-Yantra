#!/usr/bin/env python3
"""
Conducts marker detection of a moving ArUco marker.

This module performs marker detection of a moving marker in Gazebo wherein the raw data sent by the camera in the Gazebo world is then analysed and ArUco marker data is extracted.
"""

import math
from cv_bridge import CvBridge, CvBridgeError
from cv2 import aruco
import numpy as np
import rospy
from sensor_msgs.msg import Image
from task_1.msg import Marker


class ImageProc:
    """
    This class contains functions for ArUco marker processing.
    """

    # Initialise everything
    def __init__(self):
        """
        __init__ Initialising class attributes

        This function initialises the various class attributes for the ImageProc class, which contains essential methods and attributes to process ArUco markers.
        """
        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.detected_aruco_markers = {}

        # Initialise rosnode
        rospy.init_node("marker_detection", anonymous=True)
        self.rate = rospy.Rate(10)
        # This will contain the message structure of message type ss_1302_task_1/Marker
        self.marker_msg = Marker()

        # Making a publisher
        self.marker_pub = rospy.Publisher("/marker_info", Marker, queue_size=1)
        # Subscribing to the camera topic /camera/camera/image_raw
        self.image_sub = rospy.Subscriber(
            "/camera/camera/image_raw", Image, self.image_callback
        )

    def detect_aruco(self, img: np.array):
        """
        detect_aruco Detects the ArUcos in the image and extracts ID and coordinates.

        This method uses the aruco library to extract information about any detected ArUco markers in the input image data. It extracts the ID of the marker as embedded as well as the coordinates of the four markers of the ArUco marker, which can then be used to calculate the centre position of the marker and its orientation in ensuing methods.

        :param img: The image data to be analysed by the method.
        :type img: np.array
        """
        corners = []
        ids = []
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(img, aruco_dict, parameters=parameters)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (marker_corner, marker_id) in zip(corners, ids):
                self.detected_aruco_markers[marker_id] = marker_corner

    def image_callback(self, data: np.array):
        """
        image_callback Callback function of the camera topic.

        This is the callback function for the subscriber to the /camera/camera/image_raw topic. It receives raw image data from the topic in the form of an nparray, and then converts it to a format compatible with OpenCV operations.

        :param data: The raw image data from /camera/camera/image_raw topic.
        :type data: Image
        """
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            print(err)
            return

    def publish_data(self):
        """
        publish_data Publish the data to topic.

        This method publishes the data held in this node to the /marker_info topic, through the marker_pub Publisher, by using the message template specified by marker_msg.
        """
        self.marker_pub.publish(self.marker_msg)

    def marker_pos(self):
        """
        marker_pos Calculates the position and orientation of the (centre position of) the marker.

        This method calculates the position and orientation of the detected ArUco marker. It calculates the position based on the position of the centre of the marker, and the orientation based on the angle subtended by the lines joining the centre and one corner with respect to the horizontal.
        """
        for _, coods in self.detected_aruco_markers.items():
            # Calculating relevant point coordinates
            x_0, y_0 = map(int, coods[0][0])
            x_2, y_2 = map(int, coods[0][2])

            # Calculating centre position
            c_x = int((x_0 + x_2) / 2)
            c_y = int((y_0 + y_2) / 2)

            # Calculating orientation angle
        for ids, corner in self.detected_aruco_markers.items():
            corner = corner[0]
            # Since angle is atan2(-y,x), then converting that to degrees
            top_right_angle = (
                math.degrees(
                    math.atan2(
                        -corner[1][1] + corner[3][1], corner[1][0] - corner[3][0]
                    )
                )
            ) % 360
            angle = int((top_right_angle + 45) % 360)

            # Filling in message data
            self.marker_msg.id = int(ids)
            self.marker_msg.x = c_x
            self.marker_msg.y = c_y
            self.marker_msg.yaw = angle
            self.publish_data()  # Publishing data


if __name__ == "__main__":
    image_proc_obj = ImageProc()  # Creating object
    rospy.sleep(3)  # 3 sec Delay for syncing
    try:
        while not rospy.is_shutdown():  # Will run till node active
            image_proc_obj.rate.sleep()  # Delay
            # Detecting ArUco marker
            image_proc_obj.detect_aruco(image_proc_obj.img)
            # Calculating and publishing position/orientation data
            image_proc_obj.marker_pos()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
