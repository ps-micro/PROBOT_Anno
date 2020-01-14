#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import cv2
import time
import math
import os
import numpy as np
import tensorflow as tf
from math import *
import time
import os
import argparse
import re

import locality_aware_nms as nms_locality
import lanms

import model
from icdar import restore_rectangle

import roslib
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError

from probot_ocr.srv import *
FLAGS = tf.app.flags.FLAGS
global point1
global point2
global img
global area
global num
global args

area = 0
num = 0

double Pixel2Chess[3][3] = {25.33432115950898, 713.4809289445094, -344.9663160539974, 208454.8922781001,
 721.1309086833608, -23.02279647167147, -221.5281188407786, 150445.2345246944,
 0.01700523308459738, -0.01538153763047186, -0.9997370806106278, 859.2510678523352};
double Chess2Base[4][4] = {-0.9997623668389658, -0.01776013162391372, 0.01264071110540455, 205.485566828755,
 0.01806124026157617, -0.9995459301701829, 0.02411897759932594, 339.980819466093,
 0.01220661512306583, 0.02434155305079011, 0.9996291749155348, 172.6578194047057,
 0, 0, 0, 1};
Mat Pixel2ChessMatrix(3, 3, CV_64FC1, Pixel2Chess);
Mat Chess2BaseMatrix(4, 4, CV_64FC1, Chess2Base);

class TOD(object):
    def __init__(self):

        self.graph = tf.get_default_graph()
        self.input_images = tf.placeholder(tf.float32, shape=[None, None, None, 3], name='input_images')
        self.global_step = tf.get_variable('global_step', [], initializer=tf.constant_initializer(0), trainable=False)
        self.f_score, self.f_geometry = model.model(self.input_images, is_training=False)

        self.variable_averages = tf.train.ExponentialMovingAverage(0.997, self.global_step)
        self.saver = tf.train.Saver(self.variable_averages.variables_to_restore())

        self.sess = tf.InteractiveSession(config=tf.ConfigProto(allow_soft_placement=True))

        global args
        self.ckpt_state = tf.train.get_checkpoint_state(args.model_path)
        self.model_path = os.path.join(args.model_path, os.path.basename(self.ckpt_state.model_checkpoint_path))
        self.saver.restore(self.sess,self.model_path)

        self.poseList=[]
        self.angleList=[]
        self.charname = []
        self.charnum = []
        self.com = {}

    def rotate(self,img,pt1, pt2, pt3, pt4):
        print(pt1,pt2,pt3,pt4)
        withRect = math.sqrt((pt4[0] - pt1[0]) ** 2 + (pt4[1] - pt1[1]) ** 2) # 矩形框的宽度
        heightRect = math.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) **2)
        print(withRect,heightRect)
        #angle = acos((pt4[0] - pt1[0]) / withRect) * (180 / math.pi) # 矩形框旋转角度
        print("acos" ,((pt1[0] - pt4[0]) / withRect))
        angleh = math.acos((pt1[0] - pt4[0]) / withRect)
        angle = math.acos((pt1[0] - pt4[0]) / withRect) * (180 / math.pi)
        print(angle)

        if pt4[1] > pt1[1]:
            angle=-angle
            angleh=-angleh
            print("顺时针旋转")
        else:
            print("逆时针旋转")
            
        self.angleList.append(-1 * angleh)

        height = img.shape[0] # 原始图像高度
        width = img.shape[1]  # 原始图像宽度
        #angle > 0 ni angle < 0 shun
        rotateMat = cv2.getRotationMatrix2D((width / 2, height / 2), angle, 1) # 按angle角度旋转图像
        heightNew = int(width * fabs(sin(radians(angle))) + height * fabs(cos(radians(angle))))
        widthNew = int(height * fabs(sin(radians(angle))) + width * fabs(cos(radians(angle))))

        rotateMat[0, 2] += (widthNew - width) / 2
        rotateMat[1, 2] += (heightNew - height) / 2
        imgRotation = cv2.warpAffine(img, rotateMat, (widthNew, heightNew), borderValue=(255, 255, 255))
        print("x")
        global num
        num = num + 1
        global args
        cv2.imwrite(os.path.join(args.output_path,"imgRotation.jpg"),imgRotation)

        # 旋转后图像的四点坐标
        [[pt1[0]], [pt1[1]]] = np.dot(rotateMat, np.array([[pt1[0]], [pt1[1]], [1]]))
        [[pt3[0]], [pt3[1]]] = np.dot(rotateMat, np.array([[pt3[0]], [pt3[1]], [1]]))
        [[pt2[0]], [pt2[1]]] = np.dot(rotateMat, np.array([[pt2[0]], [pt2[1]], [1]]))
        [[pt4[0]], [pt4[1]]] = np.dot(rotateMat, np.array([[pt4[0]], [pt4[1]], [1]]))

        # 处理反转的情况
        if pt2[1]>pt4[1]:
            pt2[1],pt4[1]=pt4[1],pt2[1]
        if pt1[0]>pt3[0]:
            pt1[0],pt3[0]=pt3[0],pt1[0]
        imgOut = imgRotation[int(pt2[1]):int(pt4[1]), int(pt1[0]):int(pt3[0])]

        
        #cv2.namedWindow('imgOut',cv2.WINDOW_NORMAL)
        #cv2.imshow("imgOut", imgOut) # 裁减得到的旋转矩形框
        #cv2.waitKey(30)
        cv2.imwrite(os.path.join(args.output_path,"imgOut.jpg"),imgOut)


        tpl = imgOut.copy()
        src_path = args.template_path
        filenames = os.listdir(src_path)
        minest = 100.0
        for target1 in filenames:
            target = cv2.imread(src_path + target1)
            #tpl = cv.imread("/home/sijia/OCR/templet_image/cai/R.jpg")
            #cv.imshow("modul", tpl)
            #cv.imshow("yuan", target)
            methods = [cv2.TM_SQDIFF_NORMED]#, cv.TM_CCORR_NORMED, cv.TM_CCOEFF_NORMED]
            th, tw = tpl.shape[:2]
            for md in methods:
                result = cv2.matchTemplate(target, tpl, md)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

                if md == cv2.TM_SQDIFF_NORMED:
                    tl = min_loc
                    if min_val < minest:
                        minest = min_val
                        min_image = target1
                    print(min_val)
                else:
                    tl = max_loc
                    print(max_val)
                br = (tl[0] + tw, tl[1] + th)
                cv2.rectangle(target, tl, br, [0, 0, 0])

        

        self.charname.append(min_image[:-5])
        self.charnum.append(min_image[-5:-4])

        gray_image = cv2.cvtColor(imgOut.copy(), cv2.COLOR_BGR2GRAY)
        gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        ret3, th3 = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        return imgRotation # rotated image

    def on_mouse(self,event, x, y, flags, param):
        print("select the detect area")
        global point1, point2, img, area
        img2 = img.copy()
        if event == cv2.EVENT_LBUTTONDOWN:         #左键点击
            point1 = (x,y)
            cv2.circle(img2, point1, 10, (0,255,0), 5)
            cv2.imshow('image', img2)
        elif event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON):               #按住左键拖曳
            cv2.rectangle(img2, point1, (x,y), (255,0,0), 5)
            cv2.imshow('image', img2)
        elif event == cv2.EVENT_LBUTTONUP:         #左键释放
            point2 = (x,y)
            cv2.rectangle(img2, point1, point2, (0,0,255), 5) 
            cv2.imshow('image', img2)
            min_x = min(point1[0],point2[0])     
            min_y = min(point1[1],point2[1])
            width = abs(point1[0] - point2[0])
            height = abs(point1[1] -point2[1])
            cut_img = img[min_y:min_y+height, min_x:min_x+width]
            cv2.imwrite('lena3.jpg', cut_img)
            area = 1
            print("tf version",tf.__version__)


    def resize_image(self,im, max_side_len=2400):
        '''
        resize image to a size multiple of 32 which is required by the network
        :param im: the resized image
        :param max_side_len: limit of max image size to avoid out of memory in gpu
        :return: the resized image and the resize ratio
        '''
        h, w, _ = im.shape

        resize_w = w
        resize_h = h

        # limit the max side
        if max(resize_h, resize_w) > max_side_len:
            ratio = float(max_side_len) / resize_h if resize_h > resize_w else float(max_side_len) / resize_w
        else:
            ratio = 1.
        resize_h = int(resize_h * ratio)
        resize_w = int(resize_w * ratio)

        resize_h = resize_h if resize_h % 32 == 0 else (resize_h // 32 - 1) * 32
        resize_w = resize_w if resize_w % 32 == 0 else (resize_w // 32 - 1) * 32
        resize_h = max(32, resize_h)
        resize_w = max(32, resize_w)
        im = cv2.resize(im, (int(resize_w), int(resize_h)))

        ratio_h = resize_h / float(h)
        ratio_w = resize_w / float(w)

        return im, (ratio_h, ratio_w)


    def detect(self,score_map, geo_map, timer, score_map_thresh=0.8, box_thresh=0.1, nms_thres=0.2):
        '''
        restore text boxes from score map and geo map
        :param score_map:
        :param geo_map:
        :param timer:
        :param score_map_thresh: threshhold for score map
        :param box_thresh: threshhold for boxes
        :param nms_thres: threshold for nms
        :return:
        '''
        if len(score_map.shape) == 4:
            score_map = score_map[0, :, :, 0]
            geo_map = geo_map[0, :, :, ]
        # filter the score map
        xy_text = np.argwhere(score_map > score_map_thresh)
        # sort the text boxes via the y axis
        xy_text = xy_text[np.argsort(xy_text[:, 0])]
        # restore
        start = time.time()
        text_box_restored = restore_rectangle(xy_text[:, ::-1]*4, geo_map[xy_text[:, 0], xy_text[:, 1], :]) # N*4*2
        print('{} text boxes before nms'.format(text_box_restored.shape[0]))
        #print("zaina")
        boxes = np.zeros((text_box_restored.shape[0], 9), dtype=np.float32)
        boxes[:, :8] = text_box_restored.reshape((-1, 8))
        boxes[:, 8] = score_map[xy_text[:, 0], xy_text[:, 1]]
        timer['restore'] = time.time() - start

        # nms part
        start = time.time()
        # boxes = nms_locality.nms_locality(boxes.astype(np.float64), nms_thres)
        boxes = lanms.merge_quadrangle_n9(boxes.astype('float32'), nms_thres)
        timer['nms'] = time.time() - start
        
        if boxes.shape[0] == 0:
            return None, timer

        # here we filter some low score boxes by the average score map, this is different from the orginal paper
        for i, box in enumerate(boxes):
            mask = np.zeros_like(score_map, dtype=np.uint8)
            cv2.fillPoly(mask, box[:8].reshape((-1, 4, 2)).astype(np.int32) // 4, 1)
            boxes[i, 8] = cv2.mean(score_map, mask)[0]
        boxes = boxes[boxes[:, 8] > box_thresh]

        return boxes, timer


    def sort_poly(self,p):
        min_axis = np.argmin(np.sum(p, axis=1))
        p = p[[min_axis, (min_axis+1)%4, (min_axis+2)%4, (min_axis+3)%4]]
        if abs(p[0, 0] - p[1, 0]) > abs(p[0, 1] - p[1, 1]):
            return p
        else:
            return p[[0, 3, 2, 1]]

    def Iscollect(self):
        pass

    def detect_char(self,req):

        charObjList = []
        charName    = []
        charNum     = []
        angle       = []
        res =  detectobjectionSrvResponse.NOT_DETECTED


        self.charname.clear()
        self.charnum.clear()
        self.poseList.clear()
        self.angleList.clear()

        imageData = rospy.wait_for_message('/image_converter/output_video', Image, timeout=5)
        try:
            cv_image = CvBridge().imgmsg_to_cv2(imageData,"bgr8")

        except CvBridgeError as e:
            print(e)

        global args
        cv2.imwrite(args.output_path+"cv_image.jpg",cv_image)
        if req.objectType is detectobjectionSrvRequest.ALL_OBJECT:
            global img, area, point1, point2
            img = cv_image.copy()
            img2 = cv_image.copy()
            im = cv_image[:, :, ::-1]

            if area == 0:
                cv2.namedWindow('image')
                cv2.setMouseCallback('image', self.on_mouse)
            while True:
                if(area == 1):
                    break
                cv2.imshow('image', im)
                cv2.waitKey(30)
            
            cv2.destroyAllWindows()
            cv2.waitKey(15)
            area = 1
            h, w, _ = im.shape
            start_time = time.time()
            im_resized, (ratio_h, ratio_w) = self.resize_image(im)
            h, w, _ = im.shape

            timer = {'net': 0, 'restore': 0, 'nms': 0}
            start = time.time()



            score, geometry = self.sess.run([self.f_score, self.f_geometry], feed_dict={self.input_images: [im_resized]})
            timer['net'] = time.time() - start

            boxes, timer = self.detect(score_map=score, geo_map=geometry, timer=timer)

            if boxes is not None:
                boxes = boxes[:, :8].reshape((-1, 4, 2))
                boxes[:, :, 0] /= ratio_w
                boxes[:, :, 1] /= ratio_h

            duration = time.time() - start_time
            print('[timing] {}'.format(duration))

            cv2.rectangle(img2, point1, point2, (255,0,0), 5)

            cv2.imwrite(args.output_path+"img2.jpg",img2)

            boxnum = 0
            boxdic = {}
            if boxes is not None:
                i = 0
                for box in boxes:
                    if i > 9:
                        break
                    # to avoid submitting errors
                    box = self.sort_poly(box.astype(np.int32))
                    if np.linalg.norm(box[0] - box[1]) < 5 or np.linalg.norm(box[3]-box[0]) < 5:
                        continue
                    ##zai gou xuan de qu yu if()
                    x0 = box[0,0] >= point1[0] and box[0,0] <= point2[0]
                    y0 = box[0,1] >= point1[1] and box[0,1] <= point2[1]
                    x1 = box[1,0] >= point1[0] and box[1,0] <= point2[0]
                    y1 = box[1,1] >= point1[1] and box[1,1] <= point2[1]
                    x2 = box[2,0] >= point1[0] and box[2,0] <= point2[0]
                    y2 = box[2,1] >= point1[1] and box[2,1] <= point2[1]
                    x3 = box[3,0] >= point1[0] and box[3,0] <= point2[0]
                    y3 = box[3,1] >= point1[1] and box[3,1] <= point2[1]
                    if x0 and y0 and x1 and y1 and x2 and y2 and x3 and y3:
                        print("box", box[0, 0], box[0, 1], box[1, 0], box[1, 1], box[2, 0], box[2, 1], box[3, 0], box[3, 1])
                        img2 = cv_image.copy()

                        charPose = Pose()
                        charPose.position.x = int(box[3,0]+(box[1,0]-box[3,0])/2)
                        charPose.position.y = int(box[0,1]+(box[2,1]-box[0,1])/2)

                        double a1, a2, a3, a4, a5, a6;
                        double XB, YB;
                        double XW, YW, ZW;

                        a1 = Pixel2ChessMatrix.at<double>(0, 0) - Pixel2ChessMatrix.at<double>(2, 0) * charPose.position.x;
                        a2 = Pixel2ChessMatrix.at<double>(0, 1) - Pixel2ChessMatrix.at<double>(2, 1) * charPose.position.x;
                        a3 = Pixel2ChessMatrix.at<double>(0, 2) - Pixel2ChessMatrix.at<double>(2, 2) * charPose.position.x - Pixel2ChessMatrix.at<double>(2, 3) * charPose.position.x + Pixel2ChessMatrix.at<double>(0, 3);
                        a4 = Pixel2ChessMatrix.at<double>(1, 0) - Pixel2ChessMatrix.at<double>(2, 0) * charPose.position.y;
                        a5 = Pixel2ChessMatrix.at<double>(1, 1) - Pixel2ChessMatrix.at<double>(2, 1) * charPose.position.y;
                        a6 = Pixel2ChessMatrix.at<double>(1, 2) - Pixel2ChessMatrix.at<double>(2, 2) * charPose.position.y - Pixel2ChessMatrix.at<double>(2, 3) * charPose.position.y + Pixel2ChessMatrix.at<double>(1, 3);

                        YB = (a3 * a4 - a1 * a6) / (a1 * a5 - a2 * a4);
                        XB = -1 * (a2 * YB + a3) / a1;

                        XW = Chess2BaseMatrix.at<double>(0, 0) * XB + Chess2BaseMatrix.at<double>(0, 1) * YB + Chess2BaseMatrix.at<double>(0, 3);
                        YW = Chess2BaseMatrix.at<double>(1, 0) * XB + Chess2BaseMatrix.at<double>(1, 1) * YB + Chess2BaseMatrix.at<double>(1, 3);
                        ZW = Chess2BaseMatrix.at<double>(2, 0) * XB + Chess2BaseMatrix.at<double>(2, 1) * YB + Chess2BaseMatrix.at<double>(2, 3);
                        
                        charPose.position.x = XW + 29
                        charPose.position.y = XY + 16

                        self.poseList.append(charPose)

                        cv2.circle(img2, tuple(box[0]), 3, (0,255,0), 2)
                        cv2.circle(img2, tuple(box[1]), 3, (255,255,0), 2)
                        cv2.circle(img2, tuple(box[2]), 3, (255,255,255), 2)
                        cv2.circle(img2, (charPose.position.x,charPose.position.y), 3, (0,255,0), 2)

                        cv2.imwrite(args.output_path+"img3.jpg",img2)

                        ###################
                        boxdic['box%d'%boxnum] = [[box[0, 0], box[0, 1]],box[1, 0], box[1, 1],[box[2, 0], box[2, 1]],[box[3, 0], box[3, 1]]]
                        ####################
                        image_happy = cv_image.copy()
                        self.rotate(image_happy,box[1],box[2],box[3],box[0])




                        cv2.polylines(im[:, :, ::-1].copy(), [box.astype(np.int32).reshape((-1, 1, 2))], True, color=(255, 255, 0), thickness=1)
                    i = i + 1
                cv2.imwrite(args.output_path+"detection.jpg",im[:, :, ::-1])

        charObjList = self.poseList
        charName    = self.charname
        charNum     = self.charnum
        angle       = self.angleList
        print(charName)
        print('Is ok?Y/N')
        w = input()
        return detectobjectionSrvResponse(res,charObjList,charName,charNum,angle)


if __name__ == '__main__':

    rospy.init_node('Object_Detect_ocr')

    parser = argparse.ArgumentParser(description='manual to this script')
    parser.add_argument('--model_path', type=str, default = None)
    parser.add_argument('--template_path', type=str, default = None)
    parser.add_argument('--output_path', type=str, default = None)
    global args
    args = parser.parse_args()

    detect = TOD()
    ser = rospy.Service('probot_detect_ocr',detectobjectionSrv,detect.detect_char)
    rospy.loginfo("Server is ready to detect.")
    rospy.spin()

