#! /usr/bin/env python3
import os
import sys

import numpy as np
import torch
import torch.nn as nn
import roslib
import torchvision
from torchvision import datasets, models, transforms

from std_msgs.msg import String
from core_msgs.msg import markermsg

import cv2
import rospy, roslib, rospkg
from PIL import Image

from cv_bridge import CvBridge, CvBridgeError  # Add by myself


device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

data_T= transforms.Compose([
        transforms.Resize(256),
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


class catdog_cnn_network:
    def __init__(self):
        self.image_sub = rospy.Subscriber("cropped_img", markermsg, self.callback)

        # Add by myself
        # self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)

    def callback(self, data):
        for [available, num] in [[data.image1_available, 1], [data.image2_available, 2]]:
            if available == True and num == 1:
                np_arr = np.fromstring(data.cimage1.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                cv2.imshow("first_image", cv_image)
                cv2.waitKey(500)
            elif available == True and num == 2:
                np_arr = np.fromstring(data.cimage2.data, np.uint8)
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
            pub = rospy.Publisher('catdog/String', String, queue_size=1)
            for x in ['cat','dog']:
                if x ==['cat','dog'][preds]:
                    pub.publish(x)
                    if num == 1:
                        print('first image is ', x)
                    else:
                        print('second image is ', x)

    def callback_2(self, data):

        # Readmarker routine -------------------------------------------------------------------------------------------
        cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding='bgr8')

        # For debugging
        print(type(cv_image))
        rows, cols, channels = cv_image.shape
        print('cv_image.shape', rows, cols, channels)

        inputImage = np.array(cv_image)
        outputImage = np.copy(inputImage)

        markerCorners, markerIds, rejectedImgPoints = cv2.aruco.detectMarkers(inputImage, cv2.aruco.DICT_6X6_250)

        # For debugging
        print('type(markerCorners)', type(markerCorners))

        image_1 = np.zeros((256, 256, 3), dtype=cv2.CV_8UC1)  # dtype=np.uint8
        image_1[:] = (255, 255, 255)
        image_2 = np.zeros((256, 256, 3), dtype=cv2.CV_8UC1)
        image_2[:] = (255, 255, 255)

        b_image1 = True  # image 1 available
        b_image2 = True  # image 2 available

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
                    b_image2 = False0

            if b_image1:
                innerMarkerCorners = [markerCorners[indice[0]][2],
                                      markerCorners[indice[1]][3],
                                      markerCorners[indice[2]][0],
                                      markerCorners[indice[3]][1]]

                retval, H1 = cv2.findHomography(innerMarkerCorners, outImageCorners, 0)
                image_1 = cv2.warpPerspective(inputImage, H1, (255, 255))

            if b_image2:
                innerMarkerCorners = [markerCorners[indice[4]][2],
                                      markerCorners[indice[5]][3],
                                      markerCorners[indice[6]][0],
                                      markerCorners[indice[7]][1]]

                retval, H2 = cv2.findHomography(innerMarkerCorners, outImageCorners, 0)
                image_2 = cv2.warpPerspective(inputImage, H2, (255, 255))
        else:
            b_image1 = False
            b_image2 = False

        # Here in cpp file we convert image_1 & image_2 to text and publich
        # We can instead just continue procesing here (Feeding to CNN network)
        
        # cropped_img_msg.image1_available = b_image1;                                  //Set the int message image1_available as b_image1
        # cropped_img_msg.image2_available = b_image2;                                  //Set the int message image2_available as b_image2
        #
        # imencode(".jpg", image_1, cropped_img_msg.cimage1.data);                      //Set the compressed image message cimage1 as image_1
        # imencode(".jpg", image_2, cropped_img_msg.cimage2.data);                      //Set the compressed image message cimage2 as image_2
        # input_status_pub.publish(cropped_img_msg);                                    //Publish the message containing cropped images etc.
        # cmd_target_pub.publish(cmd_target);                                           //Publish the detected marker information etc.
        #
        # Mat showImg = Mat::zeros(Size(256,512),CV_8UC3);                              //Matrix to contain images to show the wrapped images
        # image_1.copyTo(showImg(Rect(0,0,image_1.cols, image_1.rows)));                //Copy image_1 to upper side of showImg
        # image_2.copyTo(showImg(Rect(0,image_1.rows,image_2.cols,image_2.rows)));      //Copy image_1 to lower side of showImg
        #
        # imshow("image", showImg);                                                     //Show showImg in the window named "image"
        # imshow("out", outputImage);                                                   //Show outputImage in the window named "out"
        #
        # moveWindow("image", outputImage.cols+50,20);                                  //Set the window location as (width of outputImage+50, 20)
        # moveWindow("out", 50,20);                                                     //Set the window location as (width of outputImage+50, 20)
        # cvWaitKey(1);

        # CNN routine --------------------------------------------------------------------------------------------------

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
            pub = rospy.Publisher('catdog/String', String, queue_size=1)
            for x in ['cat','dog']:
                if x ==['cat','dog'][preds]:
                    pub.publish(x)
                    if num == 1:
                        print('first image is ', x)
                    else:
                        print('second image is ', x)


def main(args):
    rospy.init_node('catdog_cnn_network', anonymous=False)
    cnn = catdog_cnn_network()
    rospy.spin()


def readmarker(data):
    cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding='bgr8')

    # For debugging
    rows, cols, channels = cv_image.shape

    inputImage = np.array(cv_image)
    outputImage = np.copy(inputImage)

    markerCorners, markerIds, rejectedImgPoints = cv2.aruco.detectMarkers(inputImage, cv2.aruco.DICT_6X6_250)

    # For debugging
    print('type(markerCorners)', type(markerCorners))

    image_1 = np.zeros((256, 256, 3), dtype=cv2.CV_8UC1)  # dtype=np.uint8
    image_1[:] = (255, 255, 255)
    image_2 = np.zeros((256, 256, 3), dtype=cv2.CV_8UC1)
    image_2[:] = (255, 255, 255)

    b_image1 = 1  # image 1 available
    b_image2 = 1  # image 2 available

    if len(markerIds):
        cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds)
        indice = np.zeros(8)

        for i in range(0, 4):
            temp = np.where(markerIds == i+1)[0]
            if temp != len(markerIds)-1:
                indice[i] = temp
            else:
                b_image1 = b_image1 * 0
        for i in range(4, 8):
            temp = np.where(markerIds == i + 1)[0]
            if temp != len(markerIds) - 1:
                indice[i] = temp
            else:
                b_image2 = b_image2 * 0

        if b_image1 == 1:
            innerMarkerCorners = [markerCorners[indice[0]][2],
                                  markerCorners[indice[1]][3],
                                  markerCorners[indice[2]][0],
                                  markerCorners[indice[3]][1]]

            retval, H1 = cv2.findHomography(innerMarkerCorners, outImageCorners, 0)
            image_1 = cv2.warpPerspective(inputImage, H1, (255, 255))

        if b_image2 == 1:
            innerMarkerCorners = [markerCorners[indice[4]][2],
                                  markerCorners[indice[5]][3],
                                  markerCorners[indice[6]][0],
                                  markerCorners[indice[7]][1]]

            retval, H2 = cv2.findHomography(innerMarkerCorners, outImageCorners, 0)
            image_2 = cv2.warpPerspective(inputImage, H2, (255, 255))
    else:
        b_image1 = 0
        b_image2 = 0

    # ...


if __name__ == '__main__':
    main(sys.argv)
