#!/usr/bin/env python3

import sys
import cv2
import numpy as np
import rospy, roslib, rospkg
from sensor_msgs.msg import CompressedImage


def read_cam():
    # On versions of L4T previous to L4T 28.1, flip-method=2
    # Use the Jetson onboard camera
    cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

    pub = rospy.Publisher('jetson_image/CompressedImage', CompressedImage, queue_size=1)

    if cap.isOpened():
        while True:
            ret_val, frame = cap.read()
            cv2.imshow('jetson image',frame)
            
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            pub.publish(msg)
            if cv2.waitKey(100) == 113: # if 'q' is pressed, then program will be terminated
                cv2.destroyAllWindows()
                break ;
    else:
     print("camera open failed")
    
def main(args):

    rospy.init_node('jetson_onboard_camera', anonymous=False)
    read_cam()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
