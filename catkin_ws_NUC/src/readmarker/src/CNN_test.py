#! /usr/bin/env python3
import os
import sys
import numpy as np
import torch
import torch.nn as nn
import torchvision
from torchvision import datasets, models, transforms


from readmarker.msg import markermsg 

import cv2
import rospy, roslib, rospkg
from PIL import Image



device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


data_T= transforms.Compose([
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



class catdog_cnn_network:
    def __init__(self):
        self.image_sub = rospy.Subscriber("cropped_img",markermsg,self.callback)

    def callback(self,data):
        for [available, num] in [[data.image1_available,1] ,[data.image2_available,2]]:
            if available==True and num==1:
                np_arr= np.fromstring(data.cimage1.data, np.uint8)
                cv_image= cv2.imdecode(np_arr ,cv2.IMREAD_COLOR )
                # cv2.imshow("first_image", cv_image)
                cv2.waitKey(500)
            elif available==True and num==2:
                np_arr= np.fromstring(data.cimage2.data, np.uint8)
                cv_image= cv2.imdecode(np_arr,cv2.IMREAD_COLOR )
                # cv2.imshow("second_image", cv_image)
                cv2.waitKey(500)
            else:
                continue
            cv_image=Image.fromarray(cv_image)
            input_transform=data_T(cv_image)
            input_tensor=torch.zeros([1,3, 224, 224]).to(device)
            input_tensor[0]=input_transform

            outputs = test_model(input_tensor)
            _, preds = torch.max(outputs, 1)
            # print(preds)
            for x in ['cat','dog']:
                if x ==['cat','dog'][preds]:
                    if num==1:
                        print('first image is ' ,x)
                    else:
                        print('second image is ' ,x)


def main(args):
    cnn = catdog_cnn_network()
    rospy.init_node('catdog_cnn_network', anonymous=False)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)