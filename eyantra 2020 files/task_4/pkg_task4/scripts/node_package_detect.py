#!/usr/bin/env python
'''
This script handles the detection of the colour of the package under the camera.
'''

import rospy, time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from pyzbar.pyzbar import decode


class Camera1:

    def __init__(self):
        '''
        Constructor containing essential data.
        '''
        self.package_colour = []
        self.frame = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)

    def get_qr_data(self, arg_image):
        '''
        Obtains data from the QR Code on the package.

        Parameters:
            arg_image: The image to extract data from.
        '''
        qr_result = decode(arg_image)

        if len(qr_result) > 0:
            return qr_result[0].data
        else:
            return ('NA')

    def callback(self, data):
        '''
        Callback function for camera data.
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image = cv_image
            self.frame = image

        except CvBridgeError as e:
            rospy.logerr(e)


    def calculate(self):
        '''
        Performs Img. Proc. calculations.
        '''
        if self.frame is None:
            return

        self.package_colour = []

        box_size = 130

        first_row_x = 115
        first_row_y = 300

        inter_box_x = 180
        inter_box_y = 180

        offset = 35

        for i in range(3):
            for j in range(3):

                if i==2:
                    img = self.frame[(first_row_y + i * inter_box_y - offset): (first_row_y + i * inter_box_y - offset) + box_size,
                          (first_row_x + j * inter_box_x): (first_row_x + j * inter_box_x) + box_size]

                else:
                    img = self.frame[(first_row_y + i*inter_box_y): (first_row_y + i*inter_box_y) + box_size,
                          (first_row_x + j*inter_box_x): (first_row_x + j*inter_box_x) + box_size]

                self.package_colour.append(self.get_qr_data(img))

        pkg_colour = ",".join(self.package_colour)
        pub.publish(pkg_colour)


def listener():
    '''
    The main subscriber actions.
    '''
    ic = Camera1()

    k = time.time()

    while not rospy.is_shutdown():
        ic.calculate()
        if(abs(time.time() - k) > 30):
            quit()
        rate.sleep()
    



if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/package_colour', String, queue_size=10)
        rospy.init_node('node_package_detect', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)

        listener()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
