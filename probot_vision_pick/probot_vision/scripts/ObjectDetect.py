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

# Ros libraries
import roslib
import rospy
#import moveit_commander

import detector

# Ros Messages
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from probot_vision.srv import *

import cv2
from cv_bridge import CvBridge, CvBridgeError




def handle_object_detect(req):
    rospy.loginfo("Try to detect objects...")

    redObjList    = []
    greenObjList = []
    blueObjList   = []
    blackObjList = []
    res           = DetectObjectSrvResponse.NOT_DETECTED

    HM = rospy.get_param("~hmatrix")
    XM = rospy.get_param("~xmatrix")
    image_params = rospy.get_param("~image")

    H = HM['data']
    X = XM['data']

    try:
        imageData = rospy.wait_for_message('/usb_cam/image_raw', Image, timeout=5)
        try:
            cv_image = CvBridge().imgmsg_to_cv2(imageData,"bgr8")

        except CvBridgeError as e:
            print(e)

        cv2.imshow("detect_image", cv_image)
        cv2.waitKey(3)


        if req.objectType is DetectObjectSrvRequest.RED_OBJECT:
            red_params = rospy.get_param("~redObj")
            res, redObjList = detector.myexecute(cv_image,red_params, DetectObjectSrvRequest.RED_OBJECT, H,X,image_params)
            rospy.loginfo("Detect red object over, result: " + str(res))
        elif req.objectType is DetectObjectSrvRequest.GREEN_OBJECT:
            green_params = rospy.get_param("~greenObj")
            res, greenObjList = detector.myexecute(cv_image,green_params,DetectObjectSrvRequest.GREEN_OBJECT, H,X,image_params)
            rospy.loginfo("Detect green object over, result: " + str(res))
        elif req.objectType is DetectObjectSrvRequest.BLUE_OBJECT:
            blue_params = rospy.get_param("~blueObj")
            res, blueObjList = detector.myexecute(cv_image,blue_params, DetectObjectSrvRequest.BLUE_OBJECT, H,X,image_params)
            rospy.loginfo("Detect blue object over, result: " + str(res))
        elif req.objectType is DetectObjectSrvRequest.ALL_OBJECT:
            red_params = rospy.get_param("~redObj")
            resRed, redObjList = detector.myexecute(cv_image, red_params,DetectObjectSrvRequest.RED_OBJECT,  H,X,image_params)
            rospy.loginfo("Detect red object over, result: " + str(resRed))

            green_params = rospy.get_param("~greenObj")
            resGreen, greenObjList = detector.myexecute(cv_image,green_params, DetectObjectSrvRequest.GREEN_OBJECT, H,X,image_params)
            rospy.loginfo("Detect green object over, result: " + str(resGreen))
            
            blue_params = rospy.get_param("~blueObj")
            resBlue, blueObjList = detector.myexecute(cv_image,blue_params,DetectObjectSrvRequest.BLUE_OBJECT,  H,X,image_params)
            rospy.loginfo("Detect blue object over, result: " + str(resBlue))

            if resRed is not DetectObjectSrvResponse.SUCCESS and \
               resGreen is not DetectObjectSrvResponse.SUCCESS and \
               resBlue is not DetectObjectSrvResponse.SUCCESS:
                res = DetectObjectSrvResponse.NOT_DETECTED
            else:
                res = DetectObjectSrvResponse.SUCCESS

        # Not support request
        else:
            rospy.loginfo("Not support to detect object type " + str(req.objectType))
            res = DetectObjectSrvResponse.NOT_SUPPORT
    except rospy.ROSException:
        rospy.loginfo("Timeout waiting for image data.")
        res = DetectObjectSrvResponse.TIMEOUT
    return DetectObjectSrvResponse(res, redObjList, greenObjList, blueObjList, blackObjList)

if __name__ == '__main__':
    rospy.init_node('Object_Detect')
    ser = rospy.Service('probot_detect_object', DetectObjectSrv, handle_object_detect)
    rospy.loginfo("Server is ready to detect.")
    rospy.spin()
