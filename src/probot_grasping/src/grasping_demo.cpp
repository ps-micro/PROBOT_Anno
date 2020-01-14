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

#include "probot_grasping/grasping_demo.h"

GraspingDemo::GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length, float breadth) :
    it_(n_), 
    armgroup("manipulator"), 
    grippergroup("gripper"), 
    vMng_(length, breadth)
{
  this->nh_ = n_;

  try
  {
    this->tf_camera_to_robot.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(50.0));
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  try
  {
    this->tf_camera_to_robot.lookupTransform("/base_link", "/camera_link", ros::Time(0), (this->camera_to_robot_));
  }

  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
  }

  grasp_running = false;
  
  this->pregrasp_x = pregrasp_x;
  this->pregrasp_y = pregrasp_y;
  this->pregrasp_z = pregrasp_z;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(5.0).sleep();
  ROS_INFO_STREAM("Getting into the Grasping Position....");
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);

  // Subscribe to input video feed and publish object location
  image_sub_ = it_.subscribe("/probot_anno/camera/image_raw", 1, &GraspingDemo::imageCb, this);
}

void GraspingDemo::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
  if (!grasp_running)
  {
    ROS_INFO_STREAM("Processing the Image to locate the Object...");
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // ROS_INFO("Image Message Received");
    float obj_x, obj_y;
    vMng_.get2DLocation(cv_ptr->image, obj_x, obj_y);

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

    obj_camera_frame.setZ(-obj_y);
    obj_camera_frame.setY(-obj_x);
    obj_camera_frame.setX(0.45);

    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
    std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
    std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
  }
}

void GraspingDemo::attainPosition(float x, float y, float z)
{
  // ROS_INFO("The attain position function called");

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link_2");
  visual_tools.deleteAllMarkers();

  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;

  // Starting Postion before picking
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  armgroup.setPoseTarget(target_pose1);

  /* Uncomment Following section to visualize in rviz */
  // We can print the name of the reference frame for this robot.
  // ROS_INFO("Reference frame: %s", armgroup.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  // ROS_INFO("Reference frame: %s", armgroup.getEndEffectorLink().c_str());

  // ROS_INFO("Group names: %s",  armgroup.getName().c_str());

  /*ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

   const robot_state::JointModelGroup *joint_model_group =
  armgroup.getCurrentState()->getJointModelGroup("arm");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");*/

  armgroup.move();
}

void GraspingDemo::attainObject()
{
  // ROS_INFO("The attain Object function called");
  attainPosition(obj_robot_frame.getX(), obj_robot_frame.getY(), obj_robot_frame.getZ() + 0.04);

  // Open Gripper
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();

  // Slide down the Object
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
  geometry_msgs::Pose target_pose1;

  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  target_pose1.position.z = obj_robot_frame.getZ() - 0.02;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
}

void GraspingDemo::grasp()
{
  // ROS_INFO("The Grasping function called");

  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("close");
  grippergroup.move();
}

void GraspingDemo::lift()
{
  // ROS_INFO("The lift function called");

  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  // Starting Postion after picking
  //target_pose1.position.z = target_pose1.position.z + 0.06;

  if(rand() % 2)
  {
    target_pose1.position.y = target_pose1.position.y + 0.02;
  }
  else
  {
    target_pose1.position.y = target_pose1.position.y - 0.02;
  }
  
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();

  // Open Gripper
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();

  target_pose1.position.z = target_pose1.position.z + 0.06;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
}

void GraspingDemo::goHome()
{
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  // Go to Home Position
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);
  attainPosition(homePose.pose.position.x, homePose.pose.position.y, homePose.pose.position.z);
}

void GraspingDemo::initiateGrasping()
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(3.0).sleep();

  homePose = armgroup.getCurrentPose();
  
  ROS_INFO_STREAM("Approaching the Object....");
  attainObject();

  ROS_INFO_STREAM("Attempting to Grasp the Object now..");
  grasp();

  ROS_INFO_STREAM("Lifting the Object....");
  lift();

  ROS_INFO_STREAM("Going back to home position....");
  goHome();

  grasp_running = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_grasping");
  float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
  ros::NodeHandle n;

  if (!n.getParam("probot_grasping/table_length", length))
    length = 0.3;
  if (!n.getParam("probot_grasping/table_breadth", breadth))
    breadth = 0.3;
  if (!n.getParam("probot_grasping/pregrasp_x", pregrasp_x))
    pregrasp_x = 0.20;
  if (!n.getParam("probot_grasping/pregrasp_y", pregrasp_y))
    pregrasp_y = -0.17;
  if (!n.getParam("probot_grasping/pregrasp_z", pregrasp_z))
    pregrasp_z = 0.28;

  GraspingDemo simGrasp(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth);
  ROS_INFO_STREAM("Waiting for five seconds..");

  ros::WallDuration(5.0).sleep();
  while (ros::ok())
  {
    // Process image callback
    ros::spinOnce();

    simGrasp.initiateGrasping();
  }
  return 0;
}
