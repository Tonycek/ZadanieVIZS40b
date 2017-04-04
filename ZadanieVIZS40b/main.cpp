// **********************************************************************************
//
// BSD License.
// This file is part of a Hough Transformation tutorial,
// see: http://www.keymolen.com/2013/05/hough-transformation-c-implementation.html
//
// Copyright (c) 2013, Bruno Keymolen, email: bruno.keymolen@gmail.com
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, this
// list of conditions and the following disclaimer in the documentation and/or other
// materials provided with the distribution.
// Neither the name of "Bruno Keymolen" nor the names of its contributors may be
// used to endorse or promote products derived from this software without specific
// prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// **********************************************************************************
#include <opencv2/opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string.h>
#include <iostream>
//#include <dirent.h>
//#include <unistd.h>
#include <string>
#include <map>
#include <iostream>
#include<sstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include "hough.h"
using namespace cv;

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
/*
extern FILE *stdin;
extern FILE *stdout;
extern FILE *stderr;

*/
std::string img_path = "../Data/obrazok.png";
int threshold = 0;

//void check_program_arguments(int argc) {
//	if (argc != 2) {
//		std::cout << "Error! Program usage:" << std::endl;
//		std::cout << "./circle_detect image_circles_path" << std::endl;
//		std::exit(-1);
//	}
//}
//
//void check_if_image_exist(const cv::Mat &img, const std::string &path) {
//	if (img.empty()) {
//		std::cout << "Error! Unable to load image: " << path << std::endl;
//		std::exit(-1);
//	}
//}

//void drawObject(int x, int y, Mat &frame) {
//
//	//use some of the openCV drawing functions to draw crosshairs
//	//on your tracked image!
//
//	//UPDATE:JUNE 18TH, 2013
//	//added 'if' and 'else' statements to prevent
//	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)
//
//	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
//	if (y - 25>0)
//		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
//	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
//	if (y + 25<FRAME_HEIGHT)
//		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
//	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
//	if (x - 25>0)
//		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
//	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
//	if (x + 25<FRAME_WIDTH)
//		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
//	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);
//
//	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
//
//}

double d = 127*2, Z = 9.5, D = 4;//d = 400, Z = 18.2, D = 10;//d = 100, Z = 38, D = 10; // d = 129 * 2, Z = 15, D = 10;		// d = width, Z = vzdialenostOd, D = ballsize
double f = d*Z / D;
int posun;

int main(int argc, char **argv) {
	// Usage: ./circle_detect image_circles_path
//	check_program_arguments(argc);
	int temp = 0;
	double pixelPosun = (double)45 / 640;
	double stredSirka = 335;//640 / 2;
	// Load input image
//	std::string path_image{ argv[1] };
	Mat frame;
	std::stringstream ss;
	//ss << "../Data/cervenyKruh" << temp << ".jpg";

	//while (temp < 100) {
	//	VideoCapture capture;
	//	//open capture object at location zero (default location for webcam)
	//	capture.open(1);

	//	capture >> frame;
	//	ss << "../Data/cervenyKruhXcm" << temp << ".jpg";
	//	imwrite(ss.str(), frame);
	//	temp++;

	//	cv::Mat bgr_image = cv::imread(ss.str());
	//	cv::namedWindow("ofoteny obrazok", cv::WINDOW_NORMAL);
	//	cv::imshow("ofoteny obrazok", bgr_image);


	//	ss.str(std::string());
	//	ss.clear();
	//	waitKey(1);
	//}
	//*******************************************************************
	//VideoCapture capture;
	////open capture object at location zero (default location for webcam)
	//capture.open(1);

	//capture >> frame;
	//imwrite("../Data/8_5.png", frame);
	//******************************************************************
	
	while (temp < 100)
	{
 		ss << "../Data/cervenyKruhX" << temp << ".jpg";
	
	cv::Mat bgr_image = cv::imread(ss.str());

	// Check if the image can be loaded
//	check_if_image_exist(bgr_image, path_image);

	cv::Mat orig_image = bgr_image.clone();

	cv::medianBlur(bgr_image, bgr_image, 3);


	// Convert input image to HSV
	cv::Mat hsv_image;
	cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

	int w = hsv_image.cols;
	int h = hsv_image.rows;

	// Threshold the HSV image, keep only the red pixels
	cv::Mat lower_red_hue_range;
	cv::Mat upper_red_hue_range;
	cv::Mat green_hue_range;
	//cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
	//cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

	/*cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
	cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(189, 255, 255), upper_red_hue_range);*/

	cv::inRange(hsv_image, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), lower_red_hue_range);
	cv::inRange(hsv_image, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), upper_red_hue_range);

	cv::inRange(hsv_image, cv::Scalar(70, 100, 100), cv::Scalar(120, 255, 255), green_hue_range);



	// Combine the above two images
	cv::Mat red_hue_image;
	cv::addWeighted(lower_red_hue_range,1, upper_red_hue_range, 1, 0.0, red_hue_image);

	cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

	// Use the Hough transform to detect circles in the combined threshold image
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows / 8, 100, 20, 0, 0);

	std::vector<cv::Vec3f> circlesGreen;
	cv::HoughCircles(green_hue_range, circlesGreen, CV_HOUGH_GRADIENT, 1, green_hue_range.rows / 8, 100, 20, 0, 0);
	// Loop over all detected circles and outline them on the original image
//	if (circles.size() == 0) std::exit(-1);
	for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
		cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));

		if (current_circle != 0)
			break;

		int radius = std::round(circles[current_circle][2]);

	//	putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
		double xko = circles[current_circle][0];
		double yko = circles[current_circle][1];
	//	putText(orig_image, "X:10", Point(circles[current_circle][0], circles[current_circle][1]), 2, 1, Scalar(0, 255, 0), 2);

		if (xko > stredSirka)
			posun = xko - stredSirka;

		else
			posun = (stredSirka - xko)*(-1);

	//	double bodY = posun*pixelPosun;
		double distance = (0.0000007667*pow(radius, 4) - 0.0003074*pow(radius, 3) + 0.04678*pow(radius, 2) - 3.382*pow(radius, 1) + 115.8);	//suradnica X
		if (distance > 60)
			break;

		double Dc = sqrt(pow(xko - 320, 2) + pow(yko - 240, 2));
	/*	double lenX = xko - 335;
		double lexX = (double) lenX * 45 / 670;*/
		double Dw = (distance/100)*sin(Dc*45/640*3.14159/180);		// suradnica Y

		putText(orig_image, "BodX:" + std::to_string(distance), Point(circles[current_circle][0]-40, circles[current_circle][1]), 2, 1, Scalar(0, 255, 0), 2);
		putText(orig_image, "BodY:" + std::to_string(Dw), Point(circles[current_circle][0]-40, circles[current_circle][1] + 40), 2, 1, Scalar(0, 255, 0), 2);


		printf("Kruh %d vzdialenost od kamery: %fmm,  X:%f,  Y:%f,   polomer:%d\n", (int)current_circle, distance, xko, yko, radius);
		printf("WIDTH: %d,  HEIGHT:%d, bod Y:%f\n", w,h,Dw);
		cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5);
	}

	for (size_t current_circle = 0; current_circle < circlesGreen.size(); ++current_circle) {
		cv::Point center(std::round(circlesGreen[current_circle][0]), std::round(circlesGreen[current_circle][1]));

		if (current_circle != 0)
			break;

		int radius = std::round(circlesGreen[current_circle][2]);

		//	putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
		double xko = circlesGreen[current_circle][0];
		double yko = circlesGreen[current_circle][1];
		//	putText(orig_image, "X:10", Point(circles[current_circle][0], circles[current_circle][1]), 2, 1, Scalar(0, 255, 0), 2);

		if (xko > stredSirka)
			posun = xko - stredSirka;

		else
			posun = (stredSirka - xko)*(-1);

		//	double bodY = posun*pixelPosun;
		double distance = (0.0000007667*pow(radius, 4) - 0.0003074*pow(radius, 3) + 0.04678*pow(radius, 2) - 3.382*pow(radius, 1) + 115.8);	//suradnica X
		if (distance > 60)
			break;

		double Dc = sqrt(pow(xko - 320, 2) + pow(yko - 240, 2));
		/*	double lenX = xko - 335;
		double lexX = (double) lenX * 45 / 670;*/
		double Dw = (distance / 100)*sin(Dc * 45 / 640 * 3.14159 / 180);		// suradnica Y

		putText(orig_image, "BodX:" + std::to_string(distance), Point(circlesGreen[current_circle][0] - 40, circlesGreen[current_circle][1]), 2, 1, Scalar(0, 255, 0), 2);
		putText(orig_image, "BodY:" + std::to_string(Dw), Point(circlesGreen[current_circle][0] - 40, circlesGreen[current_circle][1] + 40), 2, 1, Scalar(0, 255, 0), 2);


		printf("Kruh %d vzdialenost od kamery: %fmm,  X:%f,  Y:%f,   polomer:%d\n", (int)current_circle, distance, xko, yko, radius);
		printf("WIDTH: %d,  HEIGHT:%d, bod Y:%f\n", w, h, Dw);
		cv::circle(orig_image, center, radius, cv::Scalar(255, 0, 0), 5);
	}

	// Show images
	/*cv::namedWindow("Threshold lower image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Threshold lower image", lower_red_hue_range);
	cv::namedWindow("Threshold upper image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Threshold upper image", upper_red_hue_range);*/
	cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
	cv::imshow("Combined threshold images", red_hue_image);
	cv::namedWindow("Green images", cv::WINDOW_AUTOSIZE);
	cv::imshow("Green images", green_hue_range);
	cv::namedWindow("Detected red circles on the input image", cv::WINDOW_NORMAL);
	cv::imshow("Detected red circles on the input image", orig_image);

	/*cv::resizeWindow("Threshold lower image", 800, 600);
	cv::resizeWindow("Threshold upper image", 800, 600);
	cv::resizeWindow("Combined threshold images", 800, 600);*/
	//cv::resizeWindow("Detected red circles on the input image", 800, 600);
	cv::waitKey(1000);

	temp++;

		ss.str(std::string());
		ss.clear();
	}
	//double d = 129*2, Z=15, D = 10;
	//d*Z / D

	return 0;
}
