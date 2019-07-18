#include </usr/include/eigen3/Eigen/Eigen>
#include <stdlib.h>
#include </usr/include/eigen3/Eigen/Geometry>
#include </usr/include/eigen3/Eigen/Core>
#include <vector>
#include <math.h>


#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace cv;
using namespace Eigen;

cv::Mat cameraMatrix1(3, 3, CV_64FC1);
cv::Mat cameraMatrix2(3, 3, CV_64FC1);

cv::Mat distCoeffs1(1, 5, CV_64FC1);
cv::Mat distCoeffs2(1, 5, CV_64FC1);

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  string file_path;
  //ros::Publisher image_pub_;
  int time = 0;

 
private:

    //要转换回去的图像
    sensor_msgs::ImagePtr msg;
    

public:

	ImageConverter(string pathf)
	: it_(nh_)
	{
	// Subscrive to input video feed and publish output video feed
	// ros::NodeHandle n;
	image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&ImageConverter::imageCb, this);
	image_pub_ = it_.advertise("/image_converter/output_video", 100);
	file_path =  pathf;

	}

	~ImageConverter()
	{
	}


	void imageCb(const sensor_msgs::ImageConstPtr& msgImage)
	{
        if(time == 0)
        {
        	init();
        	time = 1;
        }

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
		Size image_size;
		Mat CorrectPic;
		image_size.width = cv_ptr->image.cols;
		image_size.height = cv_ptr->image.rows;
		CorrectImageOp(cameraMatrix1, distCoeffs1,cv_ptr->image, CorrectPic, image_size,true);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", CorrectPic).toImageMsg();

        image_pub_.publish(msg);
	/******************************************************/

	}
	void init()
	{
	    ifstream f;
	    string read_path = file_path + "calibration.txt";
	    cout << read_path << endl;
	    f.open(read_path.c_str());
	    //f.open("../catkin_ws/src/Probot_vision_pick/probot_vision/config/readresult.txt");
	    string str;
	    int variablenum = 0;
	  	while (getline(f, str))
	  	{
	  		int index = 0;
	  	    istringstream input(str);
	  	    double a;
	  	    while (input >> a)
	        {
	            if(variablenum == 0)
	            {
	            	cameraMatrix1.at<double>(index / 3, index % 3) = a;
	            }
	            else if(variablenum == 1)
	            {
	            	cameraMatrix2.at<double>(index / 3, index % 3) = a;
	            }
	            else if(variablenum == 2)
	            {
	            	distCoeffs1.at<double>(0,index) = a;
	            }
	            else
	            {
	            	distCoeffs2.at<double>(0,index) = a;
	            }
	            index++;
	        }
	        variablenum++;
	  	}
	  	cout << "cameraMatrix1" << cameraMatrix1 << endl;
	  	cout << "cameraMatrix2" << cameraMatrix2 << endl;
	  	cout << "distCoeffs1" << distCoeffs1 << endl;
	  	cout << "distCoeffs2" << distCoeffs2 << endl;
	}

	void CorrectImageOp(Mat CameraMatrix, Mat DistCoeffs, Mat SourceImage, Mat& CorrectImage, Size ImageSize, bool IsDisplay)
	{
		Mat mapx = Mat(ImageSize, CV_32FC1);
		Mat mapy = Mat(ImageSize, CV_32FC1);
		Mat R = Mat();//Mat::eye(3, 3, CV_32F);
	    cout << "b" << endl;
	    cout << CameraMatrix << endl;
	    cout << DistCoeffs << endl;
		initUndistortRectifyMap(CameraMatrix, DistCoeffs, R, getOptimalNewCameraMatrix(CameraMatrix, DistCoeffs, ImageSize, 1, ImageSize, 0), ImageSize, CV_32FC1, mapx, mapy);
		remap(SourceImage.clone(), CorrectImage, mapx, mapy, CV_INTER_LINEAR);

	    if(IsDisplay)
	    {
		    namedWindow("correctFrame");
			imshow("correctFrame", CorrectImage);
			waitKey(30);
	    }
	}


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic(argv[1]);
  ros::spin();
  return 0;
}
