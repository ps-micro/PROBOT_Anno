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
#include <iostream>
#include <vector>
#include "opencv-3.3.1-dev/opencv2/highgui/highgui.hpp"
#include "opencv-3.3.1-dev/opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

using namespace cv;
using namespace std;


	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 141;
	int iHighS = 255;

	int iLowV = 115;
	int iHighV = 210;

	int iLastX = -1;
	int iLastY = -1;

	cv::Mat imgTmp;
	cv::Mat imgLines;

	int exetime = 0;


void Init(Mat picture)
{

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	//imshow("Control".CV_WINDOW_AUTOSIZE);
	waitKey(30);


	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 255);


	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);

	Mat PI = picture;//imread("1_dd.jpg");

	imgLines = Mat::zeros(PI.size(), CV_8UC3);;
}

vector<double> detect(Mat picture)
{
	vector<double> coordinate;

	if (exetime == 0)
	{
		Init(picture);
		exetime = 1;
	}


	Mat imgHSV;

	cvtColor(picture, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	Mat imgThresholded;

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

   //morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//Calculate the moments of the thresholded image
	Moments oMoments = moments(imgThresholded);

	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	if (dArea > 10000)
	{
		//calculate the position of the ball
		double posX = dM10 / dArea;
		double posY = dM01 / dArea;

		cout << "posX  " << posX;
		cout << "   posY  " << posY << endl;

		coordinate.push_back(posX);
		coordinate.push_back(posY);

		if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
		{
			//Draw a red line from the previous point to the current point
			line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
		}

		iLastX = posX;
		iLastY = posY;
	}
	//	cout << "iLastX  " << iLastX ;
	//	cout << "   iLastY  " << iLastX << endl;

	namedWindow("Thresholded Image");
	imshow("Thresholded Image", imgThresholded); //show the thresholded image
	waitKey(30);

	picture = picture + imgLines;
	namedWindow("Original");
	imshow("Original", picture); //show the original image
	waitKey(30);


	return coordinate;
}



int main(int argc, char** argv)
{
	VideoCapture cap(1); //capture the video from webcam

	Mat frame;
	//detecttest a;

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	while (true)
	{
		cap >> frame;
		detect(frame);
	}

	return 0;
}
