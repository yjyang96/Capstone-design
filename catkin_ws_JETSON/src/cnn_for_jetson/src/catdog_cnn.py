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


# CNN ------------------------------------------------------------------------------------------------------------------
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

data_T = transforms.Compose([transforms.Resize(256),
                             transforms.CenterCrop(224),
                             transforms.ToTensor(),
                             transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])

rospack = rospkg.RosPack()
root = rospack.get_path('cnn_for_jetson')
path = root+"/src/nuelnetwork/CNN_dogcat0810.pt"


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

image_1 = np.zeros((256, 256, 3), dtype=cv2.CV_8UC1)  # dtype=np.uint8
image_1[:] = (255, 255, 255)
image_2 = np.zeros((256, 256, 3), dtype=cv2.CV_8UC1)
image_2[:] = (255, 255, 255)

b_image1 = True  # image 1 available
b_image2 = True  # image 2 available


def main(args):
    rospy.init_node('catdog_cnn', anonymous=False)

    # On versions of L4T previous to L4T 28.1, flip-method=2
    # Use the Jetson onboard camera
    cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
    pub = rospy.Publisher('catdog/String', String, queue_size=1)

    if cap.isOpened():
        while True:
            ret_val, frame = cap.read()
            cv2.imshow('jetson image', frame)
            
            # Frame processing routine ---------------------------------------------------------------------------------
            # For debugging
            print(type(frame))
            rows, cols, channels = frame.shape
            print('frame.shape', rows, cols, channels)

            inputImage = np.array(frame)
            outputImage = np.copy(inputImage)

            markerCorners, markerIds, rejectedImgPoints = cv2.aruco.detectMarkers(inputImage, cv2.aruco.DICT_6X6_250)

            # For debugging
            print('type(markerCorners)', type(markerCorners))

            if len(markerIds):
                cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds)
                indice = np.zeros(8)

                for i in range(0, 4):
                    temp = np.where(markerIds == i + 1)[0]
                    if temp != len(markerIds) - 1:
                        indice[i] = temp
                    else:
                        b_image1 = False
                for i in range(4, 8):
                    temp = np.where(markerIds == i + 1)[0]
                    if temp != len(markerIds) - 1:
                        indice[i] = temp
                    else:
                        b_image2 = False

                if b_image1:
                    innerMarkerCorners = [markerCorners[indice[0]][2],
                                          markerCorners[indice[1]][3],
                                          markerCorners[indice[2]][0],
                                          markerCorners[indice[3]][1]]

                    H1, mask = cv2.findHomography(innerMarkerCorners, outImageCorners, 0)
                    image_1 = cv2.warpPerspective(inputImage, H1, (255, 255))

                if b_image2:
                    innerMarkerCorners = [markerCorners[indice[4]][2],
                                          markerCorners[indice[5]][3],
                                          markerCorners[indice[6]][0],
                                          markerCorners[indice[7]][1]]

                    H2, mask = cv2.findHomography(innerMarkerCorners, outImageCorners, 0)
                    image_2 = cv2.warpPerspective(inputImage, H2, (255, 255))
            else:
                b_image1 = False
                b_image2 = False
            # End of frame processing routine --------------------------------------------------------------------------

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
                        pub.publish(x)
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
