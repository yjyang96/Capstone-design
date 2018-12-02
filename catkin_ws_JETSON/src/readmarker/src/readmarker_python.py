#!/usr/bin/env python3

import os
import sys
import numpy as np
import cv2
import cv2.aruco as aruco

from PIL import Image

import torch
import torch.nn as nn
import torchvision
from torchvision import datasets, models, transforms

import rospy, roslib, rospkg
from std_msgs.msg import Int8
from core_msgs.msg import markerXY



# CNN ------------------------------------------------------------------------------------------------------------------
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

data_T = transforms.Compose([#transforms.Resize(256),
                             #transforms.CenterCrop(224),
                             transforms.ToTensor(),
                             transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])

rospack = rospkg.RosPack()
root = rospack.get_path('readmarker')
path = root+"/src/CNN_dogcat_sgd_dataarg.pt"


if torch.cuda.is_available():
    test_model = torch.load(path)
else:
    test_model=torch.load(path, map_location='cpu')
print('CNN_dogcat_sgd_dataarg.pt was loaded')

def main(args):
    rospy.init_node('catdog_cnn', anonymous=False)
    catdog_pub = rospy.Publisher('iscatdog/int8', Int8, queue_size=1)
    markerXY_pub = rospy.Publisher('markerXY', markerXY, queue_size=1)
    r = rospy.Rate(5)

    cap = cv2.VideoCapture(0)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH ))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT ))

    #we don't use distortion calibration data
    # mtx = np.array([[636.792913,  0, 329.63907],[ 0, 635.580978, 238.191252],[ 0, 0, 1]])
    # dist = np.array([[0.028996, -0.136993, 0.002800, 0.000258, 0.00000]])


    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    outImageCorners = np.array([[0, 0], [224, 0], [224, 224], [0, 224]])

    while not rospy.is_shutdown():
        # Frame processing routine ---------------------------------------------------------------------------------
            
        ret, frame = cap.read()
        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)


        font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

        b_image1=0;                #Let image 1 is available
        b_image2=0;                #Let image 2 is available

        cmd_target = markerXY()
        cmd_target.width = width                            
        cmd_target.height = height
        # width = frame.cols()         #Store Image's width, height in pixels
        # height = frame.rows()

        cmd_target.marker1X = [float('nan'),float('nan'),float('nan'),float('nan')]
        cmd_target.marker1Y = [float('nan'),float('nan'),float('nan'),float('nan')]
        cmd_target.marker2X = [float('nan'),float('nan'),float('nan'),float('nan')]
        cmd_target.marker2Y = [float('nan'),float('nan'),float('nan'),float('nan')]

        image_1 = np.zeros((224,224,3), np.uint8)
        image_2 = np.zeros((224,224,3), np.uint8)
        indice = {}
        aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers

        if np.all(ids != None):
            
            for i, id_ in enumerate(ids):
                indice[id_[0]-1] = i
                if id_[0] < 5:
                    b_image1+=1
                else:
                    b_image2+=1

            if b_image1 == 4:
                innerMarkerCorners = []
                innerMarkerCorners.append(corners[indice[0]][0][2])
                innerMarkerCorners.append(corners[indice[1]][0][3])
                innerMarkerCorners.append(corners[indice[2]][0][0])
                innerMarkerCorners.append(corners[indice[3]][0][1])
                for i in range(4):
                    cmd_target.marker1X[i], cmd_target.marker1Y[i] = innerMarkerCorners[i]

                innerMarkerCorners = np.array(innerMarkerCorners)

                h1, status = cv2.findHomography(innerMarkerCorners, outImageCorners)
                image_1 = cv2.warpPerspective(frame, h1, (224, 224))

            if b_image2 == 4:
                innerMarkerCorners = []
                innerMarkerCorners.append(corners[indice[4]][0][2])
                innerMarkerCorners.append(corners[indice[5]][0][3])
                innerMarkerCorners.append(corners[indice[6]][0][0])
                innerMarkerCorners.append(corners[indice[7]][0][1])
                for i in range(4):
                    cmd_target.marker2X[i], cmd_target.marker2Y[i] = innerMarkerCorners[i]

                innerMarkerCorners = np.array(innerMarkerCorners)

                h2, status = cv2.findHomography(innerMarkerCorners, outImageCorners)
                image_2 = cv2.warpPerspective(frame, h2, (224, 224))

        b_image1 = True if b_image1 == 4 else False
        b_image2 = True if b_image2 == 4 else False

        # Display the resulting frame
        # cv2.imshow('frame',frame)
        # cv2.moveWindow('frame', 50, 20)
        
        # showImg = np.concatenate((image_1, image_2), axis=0)
        # cv2.imshow('out',showImg)
        # cv2.moveWindow('out', int(width) + 50, 20)
        # cv2.waitKey(5)
        # End of frame processing routine --------------------------------------------------------------------------

        # Here in cpp file we convert image_1 & image_2 to text and publish
        # We can instead just continue processing here (Feeding to CNN network)

        markerXY_pub.publish(cmd_target)

        # CNN routine ----------------------------------------------------------------------------------------------
        for [available, num] in [[b_image1, 1], [b_image2, 2]]:
            if available and num == 1:
                cv_image = image_1
                # cv2.imshow("first_image", cv_image)
                # cv2.waitKey(500)
            elif available and num == 2:
                cv_image = image_2
                # cv2.imshow("second_image", cv_image)
                # cv2.waitKey(500)
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

            catdog_pub.publish(Int8(int(preds)))


            # debug messeg
            # for x in ['cat', 'dog']:
            #     if x == ['cat', 'dog'][preds]:
            #         # pub.publish(x)
            #         if num == 1:
            #             print('first image is ', x)
            #         else:
            #             print('second image is ', x)
        # END CNN ------------------------------------------------------------ 
        # r.sleep()

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
