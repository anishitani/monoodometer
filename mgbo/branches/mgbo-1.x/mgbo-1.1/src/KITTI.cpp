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

#include <ESM.h>

int main(int argc, char** argv)
{
	/*
	 * Initialization options
	 */
	int seq = 4;
	int i = 0;
	bool logging = false;
	bool imaging = false;

	char seq_path[256];
	sprintf(seq_path, "/media/4EAE3065AE30482B/nishitani/usp/dataset/KITTI/%02d",
			seq);
	std::string dir(seq_path);
	char *filename = new char[256];
	bool mono_fail = false;

	char odomType[3][256] =
	{ "mono", "stereo", "esm" };
	char dataType[2][256] =
	{ "pose", "motion" };
	char filepath[2][256];
	FILE* filePose[3];
	FILE* fileMotion[3];

	if (logging)
	{
		for (int i = 0; i < 3; i++)
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
	norm_vec = Rx * norm_vec;

	std::vector<std::string> win;
	win.push_back("ESM");
	win.push_back("Mono");
	win.push_back("Stereo");

// load input images
	cv::Mat I0;
	cv::Mat I1;

	cv::Mat _I0;
	cv::Mat _I1;

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

	ESM esmOdom;

	sprintf(filename, "%06d.png", i);
	_I0 = cv::imread(dir + "/image_0/" + filename, CV_LOAD_IMAGE_GRAYSCALE);
	_I1 = cv::imread(dir + "/image_1/" + filename, CV_LOAD_IMAGE_GRAYSCALE);

	// image dimensions
	int32_t width = _I0.cols;
	int32_t height = _I0.rows;

	int32_t dims[] =
	{ width, height, width };

	last = _I0;

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
			fprintf(fileMotion[1], "%f ", stereoMotion.at<float>(k));
			fprintf(filePose[1], "%f ", stereoPose.at<float>(k));
		}
		fprintf(fileMotion[1], "\n");
		fprintf(filePose[1], "\n");

		for (int k = 0; k < 12; k++)
		{
			fprintf(fileMotion[2], "%f ", esmMotion.at<float>(k));
			fprintf(filePose[2], "%f ", esmPose.at<float>(k));
		}
		fprintf(fileMotion[2], "\n");
		fprintf(filePose[2], "\n");
	}
	/* ***************************
	 * End File
	 * **************************/

	for (; !_I0.empty();)
	{
		// convert input images to uint8_t buffer
		uint8_t* I0data = (uint8_t*) _I0.data;
		uint8_t* I1data = (uint8_t*) _I1.data;

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
			mono_fail = true;
			monoMotion = cv::Mat::eye(4, 4, CV_32F);
			TRef = _I0;
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

			if (logging)
			{
				for (int k = 0; k < 12; k++)
				{
					fprintf(fileMotion[1], "%f ", stereoMotion.at<float>(k));
					fprintf(filePose[1], "%f ", stereoPose.at<float>(k));
				}
				fprintf(fileMotion[1], "\n");
				fprintf(filePose[1], "\n");
			}
		}
		else
		{
			stereoMotion = cv::Mat::eye(4, 4, CV_32F);
			TRef = _I0;
			std::cout << "Stereo Odometer failed!" << std::endl;
		}

		if (!mono_fail)
		{
			cv::warpPerspective(last, TRef, Warp, cv::Size(sizx, sizy),
					cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

			esmMotion = cv::Mat::eye(4, 4, CV_32F);
			cv::Mat(monoMotion).copyTo(esmMotion);

			esmOdom.minSSDSE2Motion(TRef, _I0, TRef.cols, TRef.rows, K,
					norm_vec / cam_height, Warp, esmMotion);

			esmPose *= esmMotion.inv();

			if (logging)
			{
				for (int k = 0; k < 12; k++)
				{
					fprintf(fileMotion[2], "%f ", esmMotion.at<float>(k));
					fprintf(filePose[2], "%f ", esmPose.at<float>(k));
				}
				fprintf(fileMotion[2], "\n");
				fprintf(filePose[2], "\n");
			}
		}
		else
		{
			esmMotion = cv::Mat::eye(4, 4, CV_32F);
			std::cout << "ESM Odometer failed!" << std::endl;
		}

		if (imaging)
		{
			/* **********************************
			 * BEGIN TESTING
			 * **********************************/
			cv::Mat diff(3 * sizy, 3 * sizx, CV_8U);
			cv::Mat R[3], t[3];
			cv::Mat TICur[3], TIRef;

			cv::warpPerspective(last, TIRef, Warp, cv::Size(sizx, sizy),
					cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

			R[0] = monoMotion(cv::Rect(0, 0, 3, 3));
			t[0] = monoMotion(cv::Rect(3, 0, 1, 3));

			R[1] = stereoMotion(cv::Rect(0, 0, 3, 3));
			t[1] = stereoMotion(cv::Rect(3, 0, 1, 3));

			R[2] = esmMotion(cv::Rect(0, 0, 3, 3));
			t[2] = esmMotion(cv::Rect(3, 0, 1, 3));

			for (int i = 0; i < 3; i++)
			{
				cv::Rect win1(0 * sizx, i * sizy, sizx, sizy);
				cv::Rect win2(1 * sizx, i * sizy, sizx, sizy);
				cv::Rect win3(2 * sizx, i * sizy, sizx, sizy);

				H = K * (R[i] - t[i] * norm_vec.t() / cam_height) * K.inv();
				cv::warpPerspective(_I0, TICur[i], H * Warp,
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
		char key = cv::waitKey(0);
		if (key == 'q')
			break;

		last = _I0;
		sprintf(filename, "%06d.png", ++i);
		_I0 = cv::imread(dir + "/image_0/" + filename, CV_LOAD_IMAGE_GRAYSCALE);
		_I1 = cv::imread(dir + "/image_1/" + filename, CV_LOAD_IMAGE_GRAYSCALE);
	}

	// output
	std::cout << "Demo complete! Exiting ..." << std::endl;

	if (logging)
	{
		for (int i = 0; i < 3; i++)
		{
			fclose(filePose[i]);
			fclose(fileMotion[i]);
		}
	}

	// exit
	return 0;
}

