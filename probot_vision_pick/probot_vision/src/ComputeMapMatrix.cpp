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

#include <stdlib.h>
#include <vector>
#include <math.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include "probot_vision/VisionMatrix.h"

#define IMAGETIME 150

using namespace cv;
using namespace std;

int CollectWorkSpace = 0;
int CountImageNum = 0;
int MatrixIndex = 0;
vector<vector<double> > data(4);
double cameraMatrix[3][3];
double distCoeffs[1][5];
double translationvector[1][3];




class ComputeMatrix
{

public:
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher matrix_pub;
    probot_vision::VisionMatrix Pubmessage;
    //vector<vector<double> > Result;

public:

    ComputeMatrix(): it_(n)
    {
      image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&ComputeMatrix::imageCb, this);
      matrix_pub = n.advertise<probot_vision::VisionMatrix>("/probot_computematrix/computematrix",1000);

    }

    void imageCb(const sensor_msgs::ImageConstPtr& msgImage)
    {
  /**************ROS与Opencv图像转换***********************/
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msgImage, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
  /******************************************************/

      cv::imshow("Image", cv_ptr->image);
      cv::waitKey(3);
      if(CountImageNum <= IMAGETIME)
          CountImageNum++;
      
      //预留时间对准工作台
      if(CountImageNum > IMAGETIME && CollectWorkSpace == 0)
      {
           //第一遍有这个
           ROS_INFO("Are you sure this is a workbench?(Y/N)");
           //cout << "Are you sure this is a workbench?(Y/N)" << endl;
           char Input;
           cin >> Input;
           if(Input == 'N' || Input == 'n')
               CountImageNum = 0;
           else if(Input == 'Y' || Input == 'y')
           {
               cv::imshow("Image", cv_ptr->image);
               cv::waitKey(3);
               ComputeMatrixfu(cv_ptr->image);
               CollectWorkSpace = 1;
               matrix_pub.publish(Pubmessage);
           }
      }
      if(CountImageNum > IMAGETIME && CollectWorkSpace == 1)
          matrix_pub.publish(Pubmessage);

    }

    void ComputeMatrixfu(Mat workspace)
    {
        dealdata();
        //
        Mat CameraMatrix(3, 3, CV_64FC1, cameraMatrix);
        Mat DistCoeffs(1, 5, CV_64FC1, distCoeffs);
        Mat TranslationVector(1, 3, CV_64FC1, translationvector);
        Mat RotationMatrix = Quaternion2RotationMatrix(data[2][0],data[2][1],data[2][2],data[2][3]);
        Mat eye2baseMatrix = CommbinationMatrixAndVector(RotationMatrix,TranslationVector,true,true,true);

        Mat ExternMatrix44;
        Mat ExternMatrix34;
        Mat rotation_matrix;
        Mat tvecsMat;
        ExternalParameter(CameraMatrix, DistCoeffs,workspace, rotation_matrix, tvecsMat, ExternMatrix44,true);
        ExternMatrix34 = CommbinationMatrixAndVector(rotation_matrix,tvecsMat,false,false,false);
        Mat H = CameraMatrix * ExternMatrix34;
        Mat X = eye2baseMatrix * ExternMatrix44;

        for(int i = 0; i < 4; i++)
        {
          for(int j = 0; j < 4; j++)
          {
            Pubmessage.ExternalMatrix.push_back(ExternMatrix44.at<double>(i,j));
            Pubmessage.xmatrix.push_back(X.at<double>(i,j));
          }
        }
        for(int i = 0; i < 3; i++)
        {
          for(int j = 0; j < 4; j++)
          {
            Pubmessage.hmatrix.push_back(H.at<double>(i,j));
          }
        }
    }

    void dealdata()
    {
        ifstream f;
        f.open("../catkin_ws/src/PROBOT_Anno/probot_vision_pick/probot_vision/config/readresult.txt");
        //f.open("../catkin_ws/src/Probot_vision_pick/probot_vision/config/readresult.txt");
        string str;
      	while (getline(f, str))
      	{
      	    istringstream input(str);
      	    double a;
      	    while (input >> a)
            {
              data[MatrixIndex].push_back(a);
              if(MatrixIndex == 0)
                Pubmessage.cameraMatrix.push_back(a);
              if(MatrixIndex == 1)
                Pubmessage.distCoeffs.push_back(a);
            }
            MatrixIndex++;
      	}
        for(int i = 0; i < 3; i++)
        {
          for(int j = 0; j < 3; j++)
          {
            cameraMatrix[i][j] = data[0][i*3+j];
          }
        }
        for(int i = 0; i < 5; i++)
        {
          distCoeffs[0][i] = data[1][i];
        }
        for(int i = 0; i < 3; i++)
        {
          translationvector[0][i] = data[3][i];
        }
    }
    Mat Quaternion2RotationMatrix(const double x, const double y, const double z, const double w)
    {
      Mat RtoMat(3,3,CV_64FC1);
      RtoMat.at<double>(0,0) = w*w + x*x - y*y - z*z;
      RtoMat.at<double>(0,1) = 2.0 * (x*y - w*z);
      RtoMat.at<double>(0,2) = 2.0 * (x*z + w*y);

      RtoMat.at<double>(1,0) = 2.0 * (x*y + w*z);
      RtoMat.at<double>(1,1) = w*w - x*x + y*y - z*z;
      RtoMat.at<double>(1,2) = 2.0 * (y*z -w*x);

      RtoMat.at<double>(2,0) = 2.0 * (x*z - w*y);
      RtoMat.at<double>(2,1) = 2.0 * (y*z + w*x);
      RtoMat.at<double>(2,2) = w*w - x*x -y*y + z*z;

      return RtoMat;
    }
    Mat CommbinationMatrixAndVector(Mat xMatrix, Mat xVector, bool IsUnitConversion,bool IsPrint, bool IsSquareMatrix)
    {
        Mat R;
        if (IsSquareMatrix)
        {
          R = Mat(4, 4, CV_64FC1);
        }
        else
        {
          R = Mat(3, 4, CV_64FC1);
        }


        for(int i = 0; i < 3; i ++)
        {
          for(int j = 0; j < 3; j++)
          {
            R.at<double>(i,j) = xMatrix.at<double>(i,j);
          }
        }

        for (int i = 0; i < 3; i++)
        {
          if(IsUnitConversion)
          {
            R.at<double>(i, 3) = xVector.at<double>(i)*1000;
          }
          else if(!IsUnitConversion)
          {
            R.at<double>(i, 3) = xVector.at<double>(i);
          }

        }
        if(IsSquareMatrix)
        {
          R.at<double>(3, 0) = 0.0;
          R.at<double>(3, 1) = 0.0;
          R.at<double>(3, 2) = 0.0;
          R.at<double>(3, 3) = 1.0;
        }
        if (IsPrint)
        {
          ROS_INFO("CombileRotaAndRec result is:");
          
          //cout << "CombileRotaAndRec result is:" << endl;
          cout << "R = " << endl << R << endl << endl;
        }

        return R;

    }
    void VectorFloatToDouble(std::vector<Point2f> imagePoint, std::vector<Point3f> worlePoint, std::vector<Point2d> &IMAGEPoint, std::vector<Point3d> &WorldPoint)
    {
      IMAGEPoint.clear();
      WorldPoint.clear();
      for (int i = 0; i < imagePoint.size(); i++)
      {
        IMAGEPoint.push_back(cv::Point2d((double)imagePoint[i].x, (double)imagePoint[i].y));
        WorldPoint.push_back(cv::Point3d((double)worlePoint[i].x, (double)worlePoint[i].y, (double)worlePoint[i].z));
      }
    }

    void ExternalParameter(Mat CameraMatrix, Mat DistCoeffs, Mat image, Mat &rotation_matrix, Mat &tvecsMat, Mat &ExternMatrix, bool IscoutAndSave)
    {
        //图片尺寸
        Size image_size;
        //标定板上每行、每列的角点数
        Size board_size = Size(6, 9);
        //实际测量得到的标定板上每个棋盘格的大小,单位mm
        Size square_size = Size(26, 26);

        //缓存每幅图像上检测到的角点
        //opencv函数求出来的image_points_buf是float类型的
        //方格的长宽也是float类型的
        vector<Point2f> image_points_buf;
        vector<Point3f> world_points_buf;
          //脑残的opencv需要double来求外参
        vector<Point2d> image_points_buf2;
        vector<Point3d> world_points_buf2;


        image_size.width = image.cols;
        image_size.height = image.rows;
        //cout << "image_size.width = " << image_size.width << endl;
        //cout << "image_size.height = " << image_size.height << endl;

        //提取角点，填充图像数组
        if (0 == findChessboardCorners(image, board_size, image_points_buf))
        {
          ROS_INFO("can not find chessboard corners!");
          //cout << "can not find chessboard corners!\n";
        }
        else
        {
          Mat view_gray;
          cvtColor(image, view_gray, CV_BGR2GRAY);
          find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5));//亚像素精确化
          drawChessboardCorners(view_gray, board_size, image_points_buf, false);

          namedWindow("Canmera Calibration");
          imshow("Canmera Calibration", view_gray);
          waitKey(500);
        }
        //填充对应世界坐标系数组
        for (int i = 0; i < board_size.height; i++)
        {
          for (int j = 0; j < board_size.width; j++)
          {
            Point3f realPoint;
            //假设标定板放在世界坐标系中Z=0的平面上
            realPoint.x = i * square_size.width;
            realPoint.y = j * square_size.height;
            realPoint.z = 0;
            world_points_buf.push_back(realPoint);
          }
        }
        //转换坐标数值类型为求外参做准备
        VectorFloatToDouble(image_points_buf, world_points_buf, image_points_buf2, world_points_buf2);
        //求外参
        Mat rvecsMat;
        solvePnP(world_points_buf2, image_points_buf2, CameraMatrix, DistCoeffs, rvecsMat, tvecsMat);
        Rodrigues(rvecsMat, rotation_matrix);

        ExternMatrix = CommbinationMatrixAndVector(rotation_matrix, tvecsMat,false,true,true);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc,argv,"probot_computematrix");

    ComputeMatrix ic;
    ros::spin();
    return 0;

}
