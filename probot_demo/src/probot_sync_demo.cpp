/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <probot_msgs/SetOutputIO.h>
#include <probot_msgs/IOStatus.h>

bool startFlag_ = false;

void ioStatusCallback(const probot_msgs::IOStatus::ConstPtr& msg)
{
    if(msg->inputIOs[0] == 1)
        startFlag_ = true;
    else
        startFlag_ = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "probot_sync_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);

    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    ros::NodeHandle n;
    ros::Publisher ioPub = n.advertise<probot_msgs::SetOutputIO>("probot_set_output_io", 1);
    ros::Subscriber sub = n.subscribe("probot_io_status", 1, ioStatusCallback);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ROS_INFO("Start Sync Demo!");

    while(startFlag_ != true)
        usleep(1000);

    while(ros::ok())
    {
        ROS_INFO("Wait Start Flag!");

        probot_msgs::SetOutputIO ioOutput1;
        ioOutput1.ioNumber = 1;
        ioOutput1.status = 1;
        ioPub.publish(ioOutput1);

        while(startFlag_ != true)   
            usleep(1000);

        std::vector<double> A = {1.57, 0.000, 0.000, 0.000, 0.000, 0.000};
        std::vector<double> B = {1.57, 0.000, 1.000, 0.000, 0.000, 0.000};
        std::vector<double> C = {-1.000, 0.000, 1.000, 0.000, 0.000, 0.000};
        std::vector<double> D = {-1.000, 0.000, 1.000, 0.000, 1.000, 0.000};

        arm.setJointValueTarget(A);
        arm.move();
        arm.setJointValueTarget(B);
        arm.move();
        arm.setJointValueTarget(C);
        arm.move();
        arm.setJointValueTarget(D);
        arm.move();

        arm.setNamedTarget("home");
        arm.move();

        std::vector<double> position1_up = {0.25105589628219604, -0.3714791238307953, -0.5240143537521362, -3.899861258105375e-05, 0.8954282999038696, 0.2508518099784851};
        std::vector<double> position1_down = {0.25105589628219604, -0.9745101928710938, -0.4310262203216553, -0.0001481490326113999, 1.4054712057113647, 0.25096750259399414};
        std::vector<double> position2_up = {-0.5309033393859863, -0.37207022309303284, -0.524405837059021, 0.00010066419781651348, 0.896560549736023, 0.2511337101459503};
        std::vector<double> position2_down = {-0.5309033393859863, -0.9512181878089905, -0.44115084409713745, 0.00035425653913989663, 1.3924535512924194, 0.2508637011051178};

        arm.setJointValueTarget(position1_up);
        arm.move();
        arm.setJointValueTarget(position1_down);
        arm.move();

        arm.setJointValueTarget(position1_up);
        arm.move();
        arm.setJointValueTarget(position2_up);
        arm.move();
        arm.setJointValueTarget(position2_down);
        arm.move();

        arm.setJointValueTarget(position2_up);
        arm.move();

        arm.setNamedTarget("home");
        arm.move();

        // 获取当前位姿数据最为机械臂运动的起始位姿
        geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;

	    std::vector<geometry_msgs::Pose> waypoints;

        //将初始位姿加入路点列表
	    waypoints.push_back(start_pose);
	
        start_pose.position.z -= 0.2;
	    waypoints.push_back(start_pose);

        start_pose.position.x += 0.1;
	    waypoints.push_back(start_pose);

        start_pose.position.y += 0.1;
	    waypoints.push_back(start_pose);

        start_pose.position.x -= 0.1;
        start_pose.position.y -= 0.1;
	    waypoints.push_back(start_pose);

	    // 笛卡尔空间下的路径规划
	    moveit_msgs::RobotTrajectory trajectory;
	    const double jump_threshold = 0.0;
	    const double eef_step = 0.01;
	    double fraction = 0.0;
        int maxtries = 100;   //最大尝试规划次数
        int attempts = 0;     //已经尝试规划次数

        while(fraction < 1.0 && attempts < maxtries)
        {
            fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;
            
            if(attempts % 10 == 0)
                ROS_INFO("Still trying after %d attempts...", attempts);
        }
        
        if(fraction == 1)
        {   
            ROS_INFO("Path computed successfully. Moving the arm.");

	        // 生成机械臂的运动规划数据
	        moveit::planning_interface::MoveGroupInterface::Plan plan;
	        plan.trajectory_ = trajectory;

	        // 执行运动
	        arm.execute(plan);
            sleep(1);
        }
        else
        {
            ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
        }

        std::vector<double> lastPoint1 = {-0.012496701441705227, -0.41917115449905396, -0.19433578848838806, 0.018087295815348625, 2.2288901805877686, -0.013513846322894096};
        std::vector<double> lastPoint2 = {-0.012496701441705227, -0.2822195291519165, -0.05068935826420784, 0.030009545385837555, 1.9483672380447388, 2.1740968227386475};

        arm.setJointValueTarget(lastPoint1);
        arm.move();
        arm.setJointValueTarget(lastPoint2);
        arm.move();

        ioOutput1.ioNumber = 1;
        ioOutput1.status = 0;
        ioPub.publish(ioOutput1);

        // 控制机械臂先回到初始化位置
        arm.setNamedTarget("home");
        arm.move();
        sleep(1);
    }

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ros::shutdown(); 

    return 0;
}
