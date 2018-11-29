#!/usr/bin/env python3

import os
import sys

import numpy as np

import cv2
from PIL import Image

import torch
import torch.nn as nn
import torchvision
from torchvision import datasets, models, transforms

import rospy, roslib, rospkg
from std_msgs.msg import String

from readmarker.msg import markerXY


# CNN ------------------------------------------------------------------------------------------------------------------
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

data_T = transforms.Compose([#transforms.Resize(256),
                             #transforms.CenterCrop(224),
                             transforms.ToTensor(),
                             transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])

rospack = rospkg.RosPack()
root = rospack.get_path('cnn_for_jetson')
path = root + "/src/nuelnetwork/CNN_dogcat0810.pt"


if torch.cuda.is_available():
    test_model = torch.load(path)
else:
    test_model=torch.load(path, map_location='cpu')
print('CNN_dogcat0810.pt was loaded')


# Add from readmarker.cpp ----------------------------------------------------------------------------------------------
outImageCorners = [(0.0, 0.0),
                   (255.0, 0.0),
                   (255.0, 255.0),
                   (0.0, 255.0)]


def main(args):
    print('Start')
    rospy.init_node('catdog_cnn', anonymous=False)

    pub_cnn = rospy.Publisher('catdog/String', String, queue_size=1)
    pub_marker = rospy.Publisher('/markerXY', markerXY, queue_size=1)  # queue_size = ?

    # On versions of L4T previous to L4T 28.1, flip-method=2
    # Use the Jetson onboard camera
    # cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
    cap = cv2.VideoCapture(0);
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    image_1 = np.zeros((224, 224, 3), dtype=np.uint8)  # np.uint8, cv2.CV_8UC1 (Not sure)
    image_2 = np.zeros((224, 224, 3), dtype=np.uint8)
    indice = np.zeros(8)

    if cap.isOpened():
        while True:
            ret_val, frame = cap.read()
            cv2.imshow('jetson image', frame)
            
            # readmarker routine ---------------------------------------------------------------------------------------
            marker = markerXY()

            b_image1 = True  # image 1 available
            b_image2 = True  # image 2 available

            marker1X = [float('nan'), float('nan'), float('nan'), float('nan')]
            marker1Y = [float('nan'), float('nan'), float('nan'), float('nan')]
            marker2X = [float('nan'), float('nan'), float('nan'), float('nan')]
            marker2Y = [float('nan'), float('nan'), float('nan'), float('nan')]
            
            # For debugging
            print('type(frame)', type(frame))
            rows, cols, channels = frame.shape
            print('frame.shape', rows, cols, channels)

            markerCorners, markerIds, rejectedImgPoints = cv2.aruco.detectMarkers(frame, cv2.aruco.DICT_6X6_250)
            
            outputImage = np.copy(frame)
            image_1[:] = (255, 255, 255)
            image_2[:] = (255, 255, 255)

            # For debugging
            print('type(markerCorners)', type(markerCorners))
            print('type(markerIds)', type(markerIds))  # I guess array type

            if markerIds.size > 0:
                cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds)
                indice[:] = 0

                for i in range(0, 4):
                    if i+1 in markerIds:  # Future warnning
                        indice[i] = np.where(markerIds == i+1)[0]
                    else:
                        b_image1 = False
                
                for i in range(4, 8):
                    if i+1 in markerIds:
                        indice[i] = np.where(markerIds == i+1)[0]
                    else:
                        b_image2 = False

                if b_image1:
                    innerMarkerCorners = np.array([markerCorners[indice[0]][2],
                                                   markerCorners[indice[1]][3],
                                                   markerCorners[indice[2]][0],
                                                   markerCorners[indice[3]][1]])
                    for i in range(4):
                        marker1X[i], marker1Y[i] = innerMarkerCorners[i]

                    H1, mask = cv2.findHomography(innerMarkerCorners, outImageCorners, 0)
                    image_1 = cv2.warpPerspective(frame, H1, (224, 224))

                if b_image2:
                    innerMarkerCorners = np.array([markerCorners[indice[4]][2],
                                                   markerCorners[indice[5]][3],
                                                   markerCorners[indice[6]][0],
                                                   markerCorners[indice[7]][1]])
                    for i in range(4):
                        marker2X[i], marker2Y[i] = innerMarkerCorners[i]

                    H2, mask = cv2.findHomography(innerMarkerCorners, outImageCorners, 0)
                    image_2 = cv2.warpPerspective(frame, H2, (224, 224))
            else:
                b_image1 = False
                b_image2 = False
            
            marker.stamp = 0
            marker.width = width
            marker.height = height
            marker.marker1X = marker1X
            marker.marker1Y = marker1Y
            marker.marker2X = marker2X
            marker.marker2Y = marker2Y

            pub_marker.publish(marker)
            # End of readmarker routine --------------------------------------------------------------------------------

            # Here in cpp file we convert image_1 & image_2 to text and publish
            # We can instead just continue processing here (Feeding to CNN network)

            # CNN routine ----------------------------------------------------------------------------------------------
            for [available, num] in [[b_image1, 1], [b_image2, 2]]:
                if available and num == 1:
                    np_arr = image_1
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    cv2.imshow("first_image", cv_image)
                    cv2.waitKey(500)
                elif available and num == 2:
                    np_arr = image_2
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    cv2.imshow("second_image", cv_image)
                    cv2.waitKey(500)
                else:
                    continue

                cv_image = Image.fromarray(cv_image)
                input_transform = data_T(cv_image)
                input_tensor = torch.zeros([1, 3, 224, 224]).to(device)
                input_tensor[0] = input_transform

                outputs = test_model(input_tensor)
                _, preds = torch.max(outputs, 1)
                # print(preds)
                # pub = rospy.Publisher('catdog/String', String, queue_size=1)
                for x in ['cat', 'dog']:
                    if x == ['cat', 'dog'][preds]:
                        pub_cnn.publish(x)
                        if num == 1:
                            print('first image is ', x)
                        else:
                            print('second image is ', x)

            if cv2.waitKey(100) == 113:  # if 'q' is pressed, then program will be terminated
                cv2.destroyAllWindows()
                break
            # End of CNN routine ---------------------------------------------------------------------------------------
    else:
        print("camera open failed")
    
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
