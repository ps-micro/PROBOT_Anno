#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import cv2
import numpy as np
import tensorflow as tf
from help_file import label_map_util
from help_file import visualization_utils as vis_util
import sys
import time

import roslib
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

#from imagetran.msg import Calibrationimage

from probot_vision.srv import *
from cv_bridge import CvBridge, CvBridgeError


(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

model_path = os.path.join(os.getcwd(),'pbF/')
Label_path = model_path + 'my_label_map.pbtxt'
Meta_graph_path = model_path + 'model.ckpt.meta'

class TOD(object):
    def __init__(self):
        self.PATH_TO_LABELS = Label_path
        self.NUM_CLASS = 3
        self.category_index = self._load_label_map()
        #shiyixia
        self.sess = tf.InteractiveSession()
        self.saver = tf.train.import_meta_graph(Meta_graph_path)
        self.graph = tf.get_default_graph()
        self.ckpt = tf.train.get_checkpoint_state(model_path)
        self.saver.restore(self.sess,self.ckpt.model_checkpoint_path)

    def _load_label_map(self):
        label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        print('label_map',label_map)
        categories = label_map_util.convert_label_map_to_categories(label_map,
                                                                    max_num_classes=self.NUM_CLASS,
                                                                    use_display_name=True)
        print('categories',categories)
        category_index = label_map_util.create_category_index(categories)
        print('category_index',category_index)
        return category_index

    def detect2(self,req):
        rospy.loginfo("Try to detect objects...")

        HM = rospy.get_param("~hmatrix")
        XM = rospy.get_param("~xmatrix")
        CM = rospy.get_param("cameraMatrix")
        EM = rospy.get_param("eyebase")
        image_params = rospy.get_param("~image")

        H = HM['data']
        X = XM['data']
        C = CM['data']
        E = EM['data']

        #C = [592.988765, 0.0, 316.144026, 0.0, 589.679756, 244.158662,0.0, 0.0, 1.0]
        #E = [0.9999262927350819, 0.012140295651447686, 0.00014939744324133248,0.0362733301805, 0.012138358976642286, -0.9998822784140743, 0.009385603649183276, 0.289040732468, 0.0002633238591056371,-0.009383098422207622, -0.9999559430922794,0.83550686443 , 0.0, 0.0, 0.0, 1.0]
        giraffeObjList  = []
        duckObjList = []
        barrotObjList   = []
        res           = DetectObjectSrvResponse.NOT_DETECTED
        imageData = rospy.wait_for_message('/camera/color/image_raw', Image)
        try:
            cv_image = CvBridge().imgmsg_to_cv2(imageData,"bgr8")

        except CvBridgeError as e:
            print(e)

        image_np_expanded = np.expand_dims(cv_image, axis=0)
        image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        scores = self.graph.get_tensor_by_name('detection_scores:0')
        classes = self.graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.graph.get_tensor_by_name('num_detections:0')
        (boxes, scores, classes, num_detections) = self.sess.run([boxes, scores, classes, num_detections],feed_dict={image_tensor: image_np_expanded})

        imagedeal = cv_image.copy()
        imagecopy = cv_image.copy()
        imagecopyp = cv_image.copy()
        vis_util.visualize_boxes_and_labels_on_image_array(
            imagecopy,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=3)
        box_to_color_map, str_to_display = vis_util.need_location(
            imagecopy,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=3)
        cv2.imwrite("jiance.jpg",imagecopy)
        

        gray_image = cv2.cvtColor(imagedeal, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        ret3, th3 = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        cv2.imwrite("beijing.jpg",th3)

        sp = imagecopy.shape
        for box, color in box_to_color_map.items():
            ymin, xmin, ymax, xmax = box
            x = int((xmax+xmin)*sp[1]/2.0)
            y = int((ymax+ymin)*sp[0]/2.0)

            cv2.circle(imagecopy,(x, y), 2, (0, 255, 0), 3)
            label = str_to_display[box]

            caijian = th3[int(ymin*sp[0]):int(ymax*sp[0]), int(xmin*sp[1]):int(xmax*sp[1])]
            if(label[0][0] == 'p'):
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (26, 26))
                kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
            else:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (6, 6))
                kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 20))
            caijian = cv2.dilate(caijian, kernel)
            caijian = cv2.erode(caijian, kernel2)
            contours,hier = cv2.findContours(caijian, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            M = cv2.moments(contours[0])
            for cnt in contours:
                N = cv2.moments(cnt)
                if M['m00'] < N['m00']:
                    M = cv2.moments(cnt)
                    #print(M['m00'])
            
            centroid_xD = M['m10']/M['m00'] + xmin*sp[1]
            centroid_yD = M['m01']/M['m00'] + ymin*sp[0]
            centroid_x = int(M['m10']/M['m00'] + xmin*sp[1])
            centroid_y = int(M['m01']/M['m00'] + ymin*sp[0])
                
            xc = (C[4] * centroid_xD - centroid_yD * C[1] - C[2] * C[4] + C[1] * C[5])/(C[4] * C[0])
            yc = (centroid_yD - C[5])/C[4]
            zc = 1

            objPose = Pose()
            objPose.position.x = E[0] * xc + E[1] * yc + E[2] * zc + E[3]
            objPose.position.y = E[4] * xc + E[5] * yc + E[6] * zc + E[7]
            print("objPose.position.x,",objPose.position.x)
            print("objPose.position.y",objPose.position.y)

            if label[0][0] == 'p':
                barrotObjList.append(objPose)
            elif label[0][0] == 'd':
                duckObjList.append(objPose)
            else:
                giraffeObjList.append(objPose)
            res = DetectObjectSrvResponse.SUCCESS
            cv2.circle(imagecopy, (centroid_x, centroid_y), 2, (0, 255, 255), 3)
            cv2.imwrite("i.jpg",imagecopy)

        return DetectObjectSrvResponse(res, giraffeObjList, duckObjList, barrotObjList)

if __name__ == '__main__':


    rospy.init_node('Object_Detect_network')
    rospy.loginfo("Server is ready to detect.")
    detecte = TOD()
    
    ser = rospy.Service('probot_detect_object_network', DetectObjectSrv, detecte.detect2)
    rospy.loginfo("Server is ready to detect.")
    rospy.spin()   
    


