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

import roslib
import rospy
import yaml
import os
from probot_vision.msg import VisionMatrix

current_path = os.path.abspath(os.path.dirname(__file__))

def read():


    with open('../.ros/camera_info/head_camera.yaml') as fc:
        temp = yaml.load(fc.read())
        camera_matrix = temp['camera_matrix']['data']
        distortion_coefficients = temp['distortion_coefficients']['data']

    with open('../.ros/easy_handeye/PROBOT_handeyecalibration_eye_on_base.yaml') as fe:
        temp1 = yaml.load(fe.read())
        quaternion = [temp1['transformation']['qx'], temp1['transformation']['qy'],temp1['transformation']['qz'],temp1['transformation']['qw']]
        location = [temp1['transformation']['x'], temp1['transformation']['y'],temp1['transformation']['z']]

    with open(current_path + '/../config/readresult.txt', 'w') as fs:
        print current_path
        #fs.write('camera_matrix:\n')
        for i in range(len(camera_matrix)):
            fs.write(str(camera_matrix[i]) + ' ')
        fs.write('\n')
        for i in range(len(distortion_coefficients)):
            fs.write(str(distortion_coefficients[i]) + ' ')
        fs.write('\n')
        for i in range(len(quaternion)):
            fs.write(str(quaternion[i]) + ' ')
        fs.write('\n')
        for i in range(len(location)):
            fs.write(str(location[i]) + ' ')
def write(MatrixData):
    with open(current_path + '/../config/vision_config.yaml', 'a') as fw:
        data = {"cameraMatrix":{'data': MatrixData.cameraMatrix},
                "distCoeffs":{'data': MatrixData.distCoeffs},
                "ExternalMatrix":{'data': MatrixData.ExternalMatrix},
                "hmatrix":{'data': MatrixData.hmatrix},
                "xmatrix":{'data': MatrixData.xmatrix}}
        yaml.dump(data,fw)

if __name__ == '__main__':
    rospy.init_node('DealYaml')
    read()
    MatrixData = rospy.wait_for_message('/probot_computematrix/computematrix', VisionMatrix)
    write(MatrixData)
    rospy.loginfo("deal finish.")
