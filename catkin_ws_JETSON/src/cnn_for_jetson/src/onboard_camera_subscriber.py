#! /usr/bin/env python3

import sys
import numpy as np
import roslib
from sensor_msgs.msg import CompressedImage

import cv2
import rospy, roslib, rospkg

class compressedimage_subscriber:
    def __init__(self):
        self.image_sub = rospy.Subscriber("jetson_image/CompressedImage",CompressedImage,self.callback, queue_size = 1)

    def callback(self,data):
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr ,cv2.IMREAD_COLOR )
        # print(data.data.size)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)





def main(args):
    rospy.init_node('compressedimage_test_node', anonymous=False)
    compressedimage_subscriber()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
