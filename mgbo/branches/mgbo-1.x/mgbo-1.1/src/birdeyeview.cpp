/*
 * odom_validation.cpp
 *
 *  Created on: Oct 24, 2013
 *      Author: nishitani
 */

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>

#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <viso_mono.h>
#include <viso_stereo.h>

#include "ESM.h"

int main(int argc, char** argv)
{
	/*
	 * Initialization options
	 */
	int seq = 4;
	int i = 0;

	char seq_path[256];
	sprintf(seq_path, "/media/4EAE3065AE30482B/nishitani/usp/dataset/%02d",
			seq);
	std::string dir(seq_path);
	char *filename = new char[256];
	bool mono_fail = false;

	char odomType[3][256] =
	{ "mono", "stereo", "esm" };
	char dataType[2][256] =
	{ "pose", "motion" };

	cv::Mat monoMotion = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat stereoMotion = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat esmMotion = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat monoPose = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat stereoPose = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat esmPose = cv::Mat::eye(4, 4, CV_32F);

	int posx = 400, posy = 225, sizx = 250, sizy = 100;
	cv::Mat norm_vec = (cv::Mat_<float>(3, 1) << 0, -1, 0);
	cv::Mat H = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat Warp = (cv::Mat_<float>(3, 3) << 1, 0, posx, 0, 1, posy, 0, 0, 1);
	cv::Mat TRef;

	float cam_baseline = 0.537;
	float cam_height = 1.7;
	float cam_pitch = -0.03;

	cv::Mat K =
			(cv::Mat_<float>(3, 3) << 707.0912, 0, 601.8873, 0, 707.0912, 183.1104, 0, 0, 1);
	cv::Mat Rx = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, cos(cam_pitch), -sin(
			cam_pitch), 0, sin(cam_pitch), cos(cam_pitch));
	norm_vec = Rx.t() * norm_vec;

	std::vector<std::string> win;
	win.push_back("ESM");
	win.push_back("Mono");
	win.push_back("Stereo");

// load input images
	cv::Mat I0;
	cv::Mat I1;

	cv::Mat last;

	/*
	 * Mono/Stereo Odometer
	 */
	VisualOdometryMono::parameters monoParam;
	VisualOdometryStereo::parameters stereoParam;

	monoParam.calib.f = (K.at<float>(0, 0) + K.at<float>(1, 1)) / 2;
	monoParam.calib.cu = K.at<float>(0, 2);
	monoParam.calib.cv = K.at<float>(1, 2);
	monoParam.height = cam_height;
	monoParam.pitch = cam_pitch;

	stereoParam.calib.f = (K.at<float>(0, 0) + K.at<float>(1, 1)) / 2;
	stereoParam.calib.cu = K.at<float>(0, 2);
	stereoParam.calib.cv = K.at<float>(1, 2);
	stereoParam.base = cam_baseline;

	// init visual odometry
	VisualOdometryMono monoOdom(monoParam);
	VisualOdometryStereo stereoOdom(stereoParam);

	sprintf(filename, "%06d.png", i);
	I0 = cv::imread(dir + "/image_0/" + filename, CV_LOAD_IMAGE_GRAYSCALE);
	I1 = cv::imread(dir + "/image_1/" + filename, CV_LOAD_IMAGE_GRAYSCALE);

	// image dimensions
	int32_t width = I0.cols;
	int32_t height = I0.rows;

	int32_t dims[] =
	{ width, height, width };

	last = I0;

	for (; !I0.empty();)
	{
		// convert input images to uint8_t buffer
		uint8_t* I0data = (uint8_t*) I0.data;
		uint8_t* I1data = (uint8_t*) I1.data;

		if (monoOdom.process(I0data, dims))
		{
			Matrix _motion = monoOdom.getMotion();
			monoMotion = cv::Mat(_motion.m, _motion.n, CV_32F);
			for (int j = 0; j < _motion.m; j++)
			{
				for (int k = 0; k < _motion.n; k++)
				{
					monoMotion.at<float>(j, k) = _motion.val[j][k];
				}
			}

			monoPose *= monoMotion.inv();

			mono_fail = false;
		}
		else
		{
			mono_fail = true;
			monoMotion = cv::Mat::eye(4, 4, CV_32F);
			TRef = I0;
			std::cout << "Monocular Odometer failed!" << std::endl;
		}

		if (stereoOdom.process(I0data, I1data, dims))
		{
			Matrix _motion = stereoOdom.getMotion();
			stereoMotion = cv::Mat(_motion.m, _motion.n, CV_32F);
			for (int j = 0; j < _motion.m; j++)
			{
				for (int k = 0; k < _motion.n; k++)
				{
					stereoMotion.at<float>(j, k) = _motion.val[j][k];
				}
			}

			stereoPose *= stereoMotion.inv();
		}
		else
		{
			stereoMotion = cv::Mat::eye(4, 4, CV_32F);
			TRef = I0;
			std::cout << "Stereo Odometer failed!" << std::endl;
		}

		if (!mono_fail)
		{
			cv::warpPerspective(last, TRef, Warp, cv::Size(sizx, sizy),
					cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

			esmMotion = cv::Mat::eye(4, 4, CV_32F);
			cv::Mat(monoMotion).copyTo(esmMotion);

			minSSDSE2Motion(TRef, I0, TRef.cols, TRef.rows, K,
					norm_vec / cam_height, Warp, esmMotion);

			esmPose *= esmMotion.inv();
		}
		else
		{
			esmMotion = cv::Mat::eye(4, 4, CV_32F);
			TRef = I0;
			std::cout << "ESM Odometer failed!" << std::endl;
		}

		if (true)
		{
			/* **********************************
			 * BEGIN TESTING
			 * **********************************/
			cv::Mat diff(I0.rows, I0.cols, CV_8U);
			cv::Mat R, t;
			cv::Mat TICur, TIRef;

			R = esmMotion(cv::Rect(0, 0, 3, 3));
			t = esmMotion(cv::Rect(3, 0, 1, 3));

			H = K * (R - t * norm_vec.t()) * K.inv();
			cv::warpPerspective(I0, TICur, H, cv::Size(I0.cols, I0.rows),
					cv::INTER_LINEAR + cv::WARP_INVERSE_MAP,
					cv::BORDER_CONSTANT, cv::Scalar(0));
			cv::Mat di;
			cv::subtract(last, I0, di);
//
			cv::imshow("All RMS", TICur);
			/* **********************************
			 * END TESTING
			 * **********************************/

//			cv::imshow("Current", I0);
			char key = cv::waitKey(60);
			if (key == 'q')
				break;
		}
		last = I0;
		sprintf(filename, "%06d.png", ++i);
		I0 = cv::imread(dir + "/image_0/" + filename, CV_LOAD_IMAGE_GRAYSCALE);
		I1 = cv::imread(dir + "/image_1/" + filename, CV_LOAD_IMAGE_GRAYSCALE);
	}

	// output
	std::cout << "Demo complete! Exiting ..." << std::endl;

	// exit
	return 0;
}

