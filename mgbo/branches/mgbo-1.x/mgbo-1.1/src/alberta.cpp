/*
 * alberta.cpp
 *
 *  Created on: Nov 23, 2013
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
	int seq = 1;
	int i = 0;
	bool logging = false;
	bool imaging = true;

	char seq_path[256];
	sprintf(seq_path, "/home/nishitani/Windows/nishitani/usp/dataset/Radish/Alberta University/ualberta-csc-flr3-vision");
	std::string dir(seq_path);
	char *filename = new char[256];

	char odomType[2][256] =
	{ "mono", "esm" };
	char dataType[2][256] =
	{ "pose", "motion" };
	char filepath[2][256];
	FILE* filePose[2];
	FILE* fileMotion[2];

	if (logging)
	{
		for (int i = 0; i < 2; i++)
		{
			sprintf(filepath[0], "data/%s/%s_%s_%02d.dat", dataType[0],
					odomType[i], dataType[0], seq);
			sprintf(filepath[1], "data/%s/%s_%s_%02d.dat", dataType[1],
					odomType[i], dataType[1], seq);
			filePose[i] = fopen(filepath[0], "w");
			fileMotion[i] = fopen(filepath[1], "w");
			if (filePose[i] == NULL || fileMotion[i] == NULL)
			{
				printf("Failed to open files %s and %s", filepath[0],
						filepath[1]);
				exit(1);
			}
		}
	}

	cv::Mat monoMotion = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat esmMotion = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat monoPose = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat esmPose = cv::Mat::eye(4, 4, CV_32F);

	int posx = 200, posy = 320, sizx = 300, sizy = 120;
	cv::Mat norm_vec = (cv::Mat_<float>(3, 1) << 0, -1, 0);
	cv::Mat H = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat Warp = (cv::Mat_<float>(3, 3) << 1, 0, posx, 0, 1, posy, 0, 0, 1);
	cv::Mat TRef;

	float cam_height = 0.7;
	float cam_pitch = 0.0;

	cv::Mat K =
			(cv::Mat_<float>(3, 3) << 527.789027, 0, 314.504927, 0, 527.423209, 213.286319, 0, 0, 1);
	cv::Mat Rx = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, cos(cam_pitch), -sin(
			cam_pitch), 0, sin(cam_pitch), cos(cam_pitch));
	norm_vec = Rx * norm_vec;

	std::vector<std::string> win;
	win.push_back("Mono");
	win.push_back("ESM");

// load input images
	cv::Mat I0;

	cv::Mat last;

	/*
	 * Mono/Stereo Odometer
	 */
	VisualOdometryMono::parameters monoParam;

	monoParam.calib.f = (K.at<float>(0, 0) + K.at<float>(1, 1)) / 2;
	monoParam.calib.cu = K.at<float>(0, 2);
	monoParam.calib.cv = K.at<float>(1, 2);
	monoParam.height = cam_height;
	monoParam.pitch = cam_pitch;

	// init visual odometry
	VisualOdometryMono monoOdom(monoParam);

	ESM esmOdom;

	sprintf(filename, "image%04d.png", i);
	I0 = cv::imread(dir + "/" + filename, CV_LOAD_IMAGE_GRAYSCALE);

	// image dimensions
	int32_t width = I0.cols;
	int32_t height = I0.rows;

	int32_t dims[] =
	{ width, height, width };

	last = I0;

	/* ***************************
	 * File
	 * **************************/
	if (logging)
	{
		for (int k = 0; k < 12; k++)
		{
			fprintf(fileMotion[0], "%f ", monoMotion.at<float>(k));
			fprintf(filePose[0], "%f ", monoPose.at<float>(k));
		}
		fprintf(fileMotion[0], "\n");
		fprintf(filePose[0], "\n");

		for (int k = 0; k < 12; k++)
		{
			fprintf(fileMotion[1], "%f ", esmMotion.at<float>(k));
			fprintf(filePose[1], "%f ", esmPose.at<float>(k));
		}
		fprintf(fileMotion[1], "\n");
		fprintf(filePose[1], "\n");
	}
	/* ***************************
	 * End File
	 * **************************/

	std::cout << ((!I0.empty() && i<30) ? "true" : "false") << std::endl;

	for (; !I0.empty();)
	{
		// convert input images to uint8_t buffer
		uint8_t* I0data = (uint8_t*) I0.data;

		/* ************************************
		 * MONOCULAR
		 * ************************************/
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

			if (logging)
			{
				for (int k = 0; k < 12; k++)
				{
					fprintf(fileMotion[0], "%f ", monoMotion.at<float>(k));
					fprintf(filePose[0], "%f ", monoPose.at<float>(k));
				}
				fprintf(fileMotion[0], "\n");
				fprintf(filePose[0], "\n");
			}
		}
		else
		{
			monoMotion = cv::Mat::eye(4, 4, CV_32F);
			TRef = I0;
			std::cout << "Monocular Odometer failed!" << std::endl;
		}

		/* ************************************
		 * ESM
		 * ************************************/
		cv::warpPerspective(last, TRef, Warp, cv::Size(sizx, sizy),
				cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

		esmMotion = cv::Mat::eye(4, 4, CV_32F);
		cv::Mat(monoMotion).copyTo(esmMotion);

		esmOdom.minSSDSE2Motion(TRef, I0, TRef.cols, TRef.rows, K,
				norm_vec / cam_height, Warp, esmMotion);

		esmPose *= esmMotion.inv();

		if (logging)
		{
			for (int k = 0; k < 12; k++)
			{
				fprintf(fileMotion[1], "%f ", esmMotion.at<float>(k));
				fprintf(filePose[1], "%f ", esmPose.at<float>(k));
			}
			fprintf(fileMotion[1], "\n");
			fprintf(filePose[1], "\n");
		}

		if (imaging)
		{
			/* **********************************
			 * BEGIN TESTING
			 * **********************************/
			cv::Mat diff(2 * sizy, 3 * sizx, CV_8U);
			cv::Mat R[2], t[2];
			cv::Mat TICur[2], TIRef;

			cv::warpPerspective(last, TIRef, Warp, cv::Size(sizx, sizy),
					cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

			R[0] = monoMotion(cv::Rect(0, 0, 3, 3));
			t[0] = monoMotion(cv::Rect(3, 0, 1, 3));

			R[1] = esmMotion(cv::Rect(0, 0, 3, 3));
			t[1] = esmMotion(cv::Rect(3, 0, 1, 3));

			for (int i = 0; i < 2; i++)
			{
				cv::Rect win1(0 * sizx, i * sizy, sizx, sizy);
				cv::Rect win2(1 * sizx, i * sizy, sizx, sizy);
				cv::Rect win3(2 * sizx, i * sizy, sizx, sizy);

				H = K * (R[i] - t[i] * norm_vec.t() / cam_height) * K.inv();
				cv::warpPerspective(I0, TICur[i], H * Warp,
						cv::Size(sizx, sizy),
						cv::INTER_LINEAR + cv::WARP_INVERSE_MAP,
						cv::BORDER_CONSTANT, cv::Scalar(-1));

				cv::Mat mask = cv::Mat::ones(sizy, sizx, CV_8U);
				for (int k = 0; k < sizy * sizx; k++)
					if (TICur[i].at<uchar>(k) < 0)
						mask.at<uchar>(k) = 0;

				cv::Mat di;
				cv::subtract(TIRef, TICur[i], di, mask, CV_32F);
				// Root mean square error calculation
				cv::Mat di2;
				cv::pow(di, 2, di2);
				float RMS = (float) cv::mean(di2).val[0];
				RMS = std::sqrt(RMS);
				char frase[256];
				sprintf(frase, "%s %f", odomType[i], RMS);
				cv::putText(di, frase, cv::Point(3, 10),
						cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255));

				TIRef.copyTo(diff(win1));
				TICur[i].copyTo(diff(win2));
				di.copyTo(diff(win3));
			}

			cv::imshow("All RMS", diff);
			/* **********************************
			 * END TESTING
			 * **********************************/
		}
		cv::Mat Warped;
		H = K
				* (esmMotion(cv::Rect(0, 0, 3, 3))
						- esmMotion(cv::Rect(3, 0, 1, 3)) * norm_vec.t()
								/ cam_height) * K.inv();
		cv::warpPerspective(last, Warped, H, cv::Size(width, height),
				cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

		cv::imshow("Current", Warped);
		char key = cv::waitKey(10);
		if (key == 'q')
			break;

		last = I0;
		sprintf(filename, "image%04d.png", ++i);
		I0 = cv::imread(dir + "/" + filename, CV_LOAD_IMAGE_GRAYSCALE);
	}

	// output
	std::cout << "Demo complete! Exiting ..." << std::endl;

	if (logging)
	{
		for (int i = 0; i < 2; i++)
		{
			fclose(filePose[i]);
			fclose(fileMotion[i]);
		}
	}

	// exit
	return 0;
}

