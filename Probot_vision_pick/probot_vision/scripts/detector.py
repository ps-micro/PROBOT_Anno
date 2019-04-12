#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import time
from math import *

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from geometry_msgs.msg import Pose
from probot_vision.srv import *

def myexecute(image,param, objType,hmatrix,xmatrix,image_params):

    # region limit
    #image = image[image_params['detectRegionLimit'][2]:image_params['detectRegionLimit'][3], \
    #        image_params['detectRegionLimit'][0]:image_params['detectRegionLimit'][1]]
    imageHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    boundaries = [(param['lowerHSV'],param['highHSV'])]

    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        # find the colors within the specified boundaries and apply the mask
        mask = cv2.inRange(imageHSV, lower, upper)
        output = cv2.bitwise_and(imageHSV, imageHSV, mask = mask)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5, 5))
    output = cv2.erode(output, kernel)
    output = cv2.dilate(output,kernel)

    thresh = cv2.erode(output, kernel)
    thresh = cv2.dilate(thresh,kernel)
    cvImg = cv2.cvtColor(thresh, 6) #cv2.COLOR_BGR2GRAY
    npImg = np.asarray( cvImg )
    thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]

    cv2.namedWindow('thresh')
    cv2.imshow('thresh', thresh)
    cv2.waitKey(300)
    # find contours in the thresholded image

    img, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    objectList = []

    for c in cnts:
        # compute the center of the contour
        M = cv2.moments(c)
        # area = cv2.contourArea(c)
        print M["m00"]
        if int(M["m00"]) not in range(10, 1500):
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

       
        a1 = hmatrix[0] - hmatrix[8] * cX
        a2 = hmatrix[1] - hmatrix[9] * cX
        a3 = hmatrix[2] - hmatrix[10] *cX - hmatrix[11] *cX + hmatrix[3]
        a4 = hmatrix[4] - hmatrix[8] * cY
        a5 = hmatrix[5] - hmatrix[9] * cY
        a6 = hmatrix[6] - hmatrix[10] *cY - hmatrix[11] *cY + hmatrix[7]
        
        objPose = Pose()
        objPose.position.y = (a3 * a4 - a1 * a6) / (a1 * a5 - a2 * a4)
        objPose.position.x = -1 * (a2 * objPose.position.y + a3) / a1

        objPose1 = Pose()
        objPose1.position.x = (xmatrix[0]*objPose.position.x + xmatrix[1]*objPose.position.y+xmatrix[3])/1000.0
        objPose1.position.y = (xmatrix[4]*objPose.position.x + xmatrix[5]*objPose.position.y+xmatrix[7])/1000.0

        # draw the contour and center of the shape on the image
        if objType is DetectObjectSrvRequest.RED_OBJECT:
            cv2.drawContours(image, [c], -1, (0, 0, 255), 2)
        elif objType is DetectObjectSrvRequest.GREEN_OBJECT:
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
        elif objType is DetectObjectSrvRequest.BLUE_OBJECT:
            cv2.drawContours(image, [c], -1, (255, 0, 0), 2)
        elif objType is DetectObjectSrvRequest.BLACK_OBJECT:
            cv2.drawContours(image, [c], -1, (255, 255, 255), 2)
       
        cv2.circle(image, (cX, cY), 3, (255, 255, 255), -1)
        
        if objPose1.position.y < image_params['boundary']
            objectList.append(objPose1)

    cv2.namedWindow('image')
    cv2.imshow('image', image)
    cv2.waitKey(3)
    result = DetectObjectSrvResponse.ERROR
    if len(objectList) > 0:
        result = DetectObjectSrvResponse.SUCCESS
    else:
        result = DetectObjectSrvResponse.NOT_DETECTED

    # show the image
    if 1 > 0 :
        if objType is DetectObjectSrvRequest.RED_OBJECT:
          cv2.imshow("Red_Object_Detect", image)
        elif objType is DetectObjectSrvRequest.GREEN_OBJECT:
          cv2.imshow("Green_Object_Detect", image)
        elif objType is DetectObjectSrvRequest.BLUE_OBJECT:
          cv2.imshow("Blue_Object_Detect", image)
        elif objType is DetectObjectSrvRequest.BLACK_OBJECT:
          cv2.imshow("Black_Object_Detect", image)
        cv2.waitKey(100)

    return result, objectList   

def execute(image, objType, params, image_params):
    #### direct conversion to CV2 ####
    np_arr = np.fromstring(image, np.uint8)
    image = cv2.imdecode(np_arr, 1) #cv2.CV_LOAD_IMAGE_COLOR

    # region limit
    #image = image[image_params['detectRegionLimit'][2]:image_params['detectRegionLimit'][3], \
    #        image_params['detectRegionLimit'][0]:image_params['detectRegionLimit'][1]]

    colorBGR = params['objectColorBGR']
    B_low = int(colorBGR[0])-params['detectBoundary']
    G_low = int(colorBGR[1])-params['detectBoundary']
    R_low = int(colorBGR[2])-params['detectBoundary']
    B_up  = int(colorBGR[0])+params['detectBoundary']
    G_up  = int(colorBGR[1])+params['detectBoundary']
    R_up  = int(colorBGR[2])+params['detectBoundary']

    if B_low < 0:B_low=0
    if G_low < 0:G_low=0
    if R_low < 0:R_low=0
    if B_up > 255:B_up=255
    if G_up > 255:G_up=255
    if R_up > 255:R_up=255

    # define the list of boundaries in BGR
    boundaries = [([B_low,G_low,R_low],[B_up,G_up,R_up])]

    # loop over the boundaries
    # print(boundaries)
    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        # find the colors within the specified boundaries and apply the mask
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)

    cvImg = cv2.cvtColor(output, 6) #cv2.COLOR_BGR2GRAY
    npImg = np.asarray( cvImg )
    thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]

    # find contours in the thresholded image
    img, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    #cnts = cnts[0]

    objectList = []
    # loop over the contours
    for c in cnts:
        # compute the center of the contour
        M = cv2.moments(c)
        # area = cv2.contourArea(c)
        if int(M["m00"]) not in range(image_params['objectSizeLimit'][0], image_params['objectSizeLimit'][1]):
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # draw the contour and center of the shape on the image
        if objType is DetectObjectSrvRequest.RED_OBJECT:
            cv2.drawContours(image, [c], -1, (0, 0, 255), 2)
        elif objType is DetectObjectSrvRequest.GREEN_OBJECT:
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
        elif objType is DetectObjectSrvRequest.BLUE_OBJECT:
            cv2.drawContours(image, [c], -1, (255, 0, 0), 2)
        elif objType is DetectObjectSrvRequest.BLACK_OBJECT:
            cv2.drawContours(image, [c], -1, (255, 255, 255), 2)
       
        cv2.circle(image, (cX, cY), 3, (255, 255, 255), -1)
        objPose = Pose()

        objPose.position.x = image_params['zeroPosition'][0] + (float(cY) + image_params['detectRegionLimit'][2]) * image_params['distanceFactor'][0]
        objPose.position.y = image_params['zeroPosition'][1] + (float(cX) + image_params['detectRegionLimit'][0]) * image_params['distanceFactor'][1]
        objectList.append(objPose)

        cv2.putText(image, str(cX + image_params['detectRegionLimit'][0])+","+str(cY + image_params['detectRegionLimit'][2]), (cX - 35, cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        #cv2.putText(image, str(objPose.position.x)[:8]+","+str(objPose.position.y)[:8], (cX - 35, cY - 15),
        #    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # print M['m00']
        # print cX, cY
        if image_params['debug'] > 0 :
            print(objPose.position.x, objPose.position.y)

    result = DetectObjectSrvResponse.ERROR
    if len(objectList) > 0:
        result = DetectObjectSrvResponse.SUCCESS
    else:
        result = DetectObjectSrvResponse.NOT_DETECTED

    # show the image
    if image_params['debug'] > 0 :
        if objType is DetectObjectSrvRequest.RED_OBJECT:
          cv2.imshow("Red_Object_Detect", image)
        elif objType is DetectObjectSrvRequest.GREEN_OBJECT:
          cv2.imshow("Green_Object_Detect", image)
        elif objType is DetectObjectSrvRequest.BLUE_OBJECT:
          cv2.imshow("Blue_Object_Detect", image)
        elif objType is DetectObjectSrvRequest.BLACK_OBJECT:
          cv2.imshow("Black_Object_Detect", image)
        cv2.waitKey(100)
    #ISOTIMEFORMAT='/home/hcx/catkin_ws/src/probot_ros/probot_packages/probot_vision/pic/%Y-%m-%d %X.png'
    #file_name =  time.strftime(ISOTIMEFORMAT, time.localtime() )
    #cv2.imwrite(file_name, image)

    return result, objectList
