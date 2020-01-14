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

from probot_vision.srv import *
from probot_msgs.msg import SetOutputIO

pickparam = rospy.get_param("/probot_sorting/pick")
placeparam = rospy.get_param("/probot_sorting/place")

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

        # Initialize IO
        self.ioPub = rospy.Publisher('probot_set_output_io', SetOutputIO, queue_size=1)

        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

    def moveToHome(self):
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

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
        self.moveTo(x, y, pickparam['hight'])

        ioOutput0 = SetOutputIO()
        ioOutput0.ioNumber = 1
        ioOutput0.status = SetOutputIO.IO_HIGH
        self.ioPub.publish(ioOutput0)
        rospy.sleep(1)
        ioOutput1 = SetOutputIO()
        ioOutput1.ioNumber = 2
        ioOutput1.status = SetOutputIO.IO_HIGH
        self.ioPub.publish(ioOutput1)

        rospy.sleep(1)

    def place(self, x, y):
        self.moveTo(x,y,placeparam['placehight'])
        rospy.sleep(0.3)

        ioOutput0 = SetOutputIO()
        ioOutput0.ioNumber = 1
        ioOutput0.status = SetOutputIO.IO_LOW
        self.ioPub.publish(ioOutput0)
        rospy.sleep(1)

        ioOutput1 = SetOutputIO()
        ioOutput1.ioNumber = 2
        ioOutput1.status = SetOutputIO.IO_LOW
        self.ioPub.publish(ioOutput1)
        rospy.sleep(1)

    def shutdown(self):
        print "The sorting is shutting down"
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)



if __name__=="__main__":
    
    time.sleep(5)
    rospy.init_node('probot_sorting')
    rate = rospy.Rate(10)

    rospy.loginfo("probot sorting start...")
    sort = ProbotSorting()

    while not rospy.is_shutdown():
        rospy.wait_for_service('probot_detect_object')
        try:
            object_detect_service = rospy.ServiceProxy('probot_detect_object',DetectObjectSrv)
            response = object_detect_service(DetectObjectSrvRequest.ALL_OBJECT)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if response.result is not DetectObjectSrvResponse.SUCCESS:
            rospy.loginfo("No objects detected , waiting detecting.....")
            rate.sleep()
            continue

        rospy.loginfo("Get object position, Start pick and place")

        if len(response.redObjList):
            sort.moveToHome()
            sort.pick(response.redObjList[0].position.x,response.redObjList[0].position.y)
            sort.moveTo(response.redObjList[0].position.x,response.redObjList[0].position.y, pickparam['liftHight'])
            sort.place(placeparam['redBox'][0], placeparam['redBox'][1])
        if len(response.greenObjList):
            sort.moveToHome()
            sort.pick(response.greenObjList[0].position.x, response.greenObjList[0].position.y)
            sort.moveTo(response.greenObjList[0].position.x,   response.greenObjList[0].position.y, pickparam['liftHight'])
            sort.place(placeparam['greenBox'][0], placeparam['greenBox'][1])
        #print response.blueObjList[0].position.x
        if len(response.blueObjList):
            sort.moveToHome()
            sort.pick(response.blueObjList[0].position.x,  response.blueObjList[0].position.y)
            sort.moveTo(response.blueObjList[0].position.x,  response.blueObjList[0].position.y, pickparam['liftHight'])
            sort.place(placeparam['blueBox'][0], placeparam['blueBox'][1])
        
        sort.moveToHome()
        rate.sleep()

    sort.moveToHome()
    sort.shutdown()




