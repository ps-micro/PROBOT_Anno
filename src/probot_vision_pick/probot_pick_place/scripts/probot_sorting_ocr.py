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

import rospy, sys
import moveit_commander
import time



from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from probot_ocr.srv import *
from probot_msgs.msg import SetOutputIO


putlocation={'H':(0.37536,0.13248),'E':(0.37536,0.053068),'L':(0.37536,-0.0264),'LL':(0.37536,-0.097946),'O':(0.37536,-0.17164),'R':(0.27466,0.12248),'OO':(0.27466,0.033957),'S':(0.27466,-0.052188),'I':(0.27466,-0.12611)}

needangle = {0:0,1:1.57,2:-3.14,3:-1.57}



class ProbotSorting:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        self.end_effector_link = self.arm.get_end_effector_link()
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)

        # 初始化场景对象
        self.scene = PlanningSceneInterface()
        rospy.sleep(1)

        # Initialize IO
        self.ioPub = rospy.Publisher('probot_set_output_io', SetOutputIO, queue_size=1)
        

        # 移除场景中之前运行残留的物体
        #scene.remove_attached_object(end_effector_link, 'tool')
        self.scene.remove_world_object('table') 
        self.scene.remove_world_object('target')

        # 设置桌面的高度
        self.table_ground = 0.05
        # 设置table和tool的三维尺寸
        self.table_size = [0.04, 0.04, 1.2]
        # 将table加入场景当中
        self.table_pose = PoseStamped()
        self.table_pose.header.frame_id = 'base_link'
        self.table_pose.pose.position.x = -0.2
        self.table_pose.pose.position.y = 0.2
        self.table_pose.pose.position.z =  0.2
        self.table_pose.pose.orientation.w = 1.0
        self.scene.add_box('table', self.table_pose, self.table_size)
        
        rospy.sleep(2)  
        # 更新当前的位姿
        self.arm.set_start_state_to_current_state()
        #########

    def moveToHome(self):
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

    def moveTo2(self,x,y,z,dx,dy,dz,dw):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = dx
        target_pose.pose.orientation.y = dy
        target_pose.pose.orientation.z = dz
        target_pose.pose.orientation.w = dw

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)

        traj = self.arm.plan()
        self.arm.execute(traj)

    def moveTo(self, x, y, z):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = 0.70692
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0.70729

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)

        traj = self.arm.plan()
        self.arm.execute(traj)

    def moveSingleJoint(self, index, value):
        group_variable_values = self.arm.get_current_joint_values()
        group_variable_values[index] = value
        self.arm.set_joint_value_target(group_variable_values)
        traj = self.arm.plan()
        self.arm.execute(traj)

    def pick(self, x, y):
        self.moveTo(x, y,0.21)
        ioOutput0 = SetOutputIO()
        ioOutput0.ioNumber = 1
        ioOutput0.status = SetOutputIO.IO_HIGH
        self.ioPub.publish(ioOutput0)
        rospy.sleep(1)


    def tai(self, x, y):

        self.moveTo(x, y, 0.28)
        ioOutput0 = SetOutputIO()
        ioOutput0.ioNumber = 1
        ioOutput0.status = SetOutputIO.IO_HIGH
        self.ioPub.publish(ioOutput0)
        rospy.sleep(1)



    def place(self, x, y):

        self.ro(1.57,5)
        pose = self.arm.get_current_pose(self.end_effector_link)
        print pose
        self.moveTo2(x,y,0.27,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)

        ioOutput0 = SetOutputIO()
        ioOutput0.ioNumber = 1
        ioOutput0.status = SetOutputIO.IO_LOW
        self.ioPub.publish(ioOutput0)
        rospy.sleep(1)


    def shutdown(self):
        print "The sorting is shutting down"
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def qiren(self):
        group_variable_values = self.arm.get_current_joint_values()
        print group_variable_values[0]
        deta = group_variable_values[0] - 7.272198854479939e-05
        a = group_variable_values[5] - deta
        print a
        if a > 3.14:
            a = -(3.1415926*2 - a)
        if a < -3.14:
            a = (3.1415926*2 + a)
        joint_positions = [7.272198854479939e-05,group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4],a]
        self.arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
        self.arm.go()
        rospy.sleep(1)

    def ro(self,angle,num):
        group_variable_values = self.arm.get_current_joint_values()
        #print group_variable_values
        group_variable_values[num] = group_variable_values[num]+angle
        if group_variable_values[num] > 3.14:
            group_variable_values[num] = -(3.1415926*2 - group_variable_values[num])
        if group_variable_values[num] < -3.14:
            group_variable_values[num] = (3.1415926*2 + group_variable_values[num])
        self.arm.set_joint_value_target(group_variable_values)
        traj = self.arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj)
        rospy.sleep(1) 

if __name__=="__main__":
    
    time.sleep(5)
    rospy.init_node('probot_sorting')
    rate = rospy.Rate(10)

    rospy.loginfo("probot sorting start...")
    sort = ProbotSorting()

    while not rospy.is_shutdown():
        rospy.wait_for_service('probot_detect_ocr')
        try:
            object_detect_service = rospy.ServiceProxy('probot_detect_ocr',detectobjectionSrv)
            response = object_detect_service(detectobjectionSrvRequest.ALL_OBJECT)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print response.result
        
        if response.result is not detectobjectionSrvResponse.SUCCESS:
            rospy.loginfo("No objects detected , waiting detecting.....")
            rate.sleep()
            continue
        
        rospy.loginfo("Get object position, Start pick and place")

        print response.charObjList
        print response.charName
        print response.charNum
        print response.angle

        L = len(response.charObjList)
        for i in range(0,L):
            sort.moveToHome()
            sort.ro(1.57,0)
            sort.pick(response.charObjList[i].position.x,response.charObjList[i].position.y)
            sort.tai(response.charObjList[i].position.x,response.charObjList[i].position.y)
            sort.ro(response.angle[i],5)
            sort.ro(needangle[int(response.charNum[i])],5)
            sort.qiren()
            sort.place(putlocation[response.charName[i]][0],putlocation[response.charName[i]][1])
            sort.moveToHome()
        

        rate.sleep()

    sort.moveToHome()
    sort.shutdown()




