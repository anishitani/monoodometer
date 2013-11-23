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

#include <camera_tool.hpp>

cv::Mat get_camera_pose(cv::Mat &I, cv::Mat K)
{
	cv::Size size(8, 6); //width,height

	cv::Mat ChessRot = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 0, 1, 0, 1, 0);

	cv::Mat corners;
	cv::Mat chess3d = CameraTool::chessboard3D_points(size.height, size.width,
			54);
	chess3d.col(1) += 70;
	chess3d.col(0) += 60;
	chess3d = (ChessRot * chess3d.t()).t();
	bool found = cv::findChessboardCorners(I, size, corners);
	if (!found)
		return cv::Mat();

	/*
	 * This line is a special case.
	 * The need to flip the corner order depends on the
	 * findChessboardCorners function.
	 */
	cv::Point first = corners.at<cv::Point>(0);
	cv::Point last = corners.at<cv::Point>(corners.rows - 1);
	if (first.x > last.x && first.y > last.y)
		cv::flip(corners, corners, 0);

	cv::Mat pose = CameraTool::cameraPoseFromMatches(corners, chess3d, K);
	pose.convertTo(pose, CV_32F);

	return pose;
}

int main(int argc, char** argv)
{
	std::string dir("/media/4EAE3065AE30482B/nishitani/usp/dataset/04");
	char *filename = new char[256];

	cv::Mat motion_stereo, motion_mono;
	cv::Mat pose_stereo = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat pose_mono = cv::Mat::eye(4, 4, CV_32F);

	float cam_height = 1.65;
	float cam_baseline = 0.5707;

	cv::Mat H = cv::Mat::eye(3, 3, CV_32F);

	cv::Mat K0 =
			(cv::Mat_<float>(3, 3) << 7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02, 0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00);
	cv::Mat K1 =
			(cv::Mat_<float>(3, 3) << 7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02, 0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00);

	std::vector<std::string> win;
	win.push_back("Image Left");
	win.push_back("Image Right");
	win.push_back("Image Difference");

	cv::namedWindow(win[0], CV_WINDOW_AUTOSIZE + CV_WINDOW_KEEPRATIO);
	cv::namedWindow(win[1], CV_WINDOW_AUTOSIZE + CV_WINDOW_KEEPRATIO);
	cv::namedWindow(win[2], CV_WINDOW_AUTOSIZE + CV_WINDOW_KEEPRATIO);

	// load input images
	cv::Mat I0; // = cv::imread("/home/nishitani/usp/dataset/image/srv/11/1.jpg");
	cv::Mat I1; // = cv::imread("/home/nishitani/usp/dataset/image/srv/11/3.jpg");

	cv::Mat _I0;
	cv::Mat _I1;

	cv::Mat last;

	/*
	 * Monocular Odometer
	 */
	VisualOdometryMono::parameters param_mono;

	param_mono.calib.f = (K0.at<float>(0, 0) + K0.at<float>(1, 1)) / 2;
	param_mono.calib.cu = K0.at<float>(0, 2);
	param_mono.calib.cv = K0.at<float>(1, 2);
	param_mono.height = cam_height;
	param_mono.pitch = 0;

	// init visual odometry
	VisualOdometryMono mono_odom(param_mono);

	/*
	 * Stereo Odometer
	 */
	VisualOdometryStereo::parameters param_stereo;

	param_stereo.calib.f = (K0.at<float>(0, 0) + K0.at<float>(1, 1)) / 2;
	param_stereo.calib.cu = K0.at<float>(0, 2);
	param_stereo.calib.cv = K0.at<float>(1, 2);
	param_stereo.base = cam_baseline;

	// init visual odometry
	VisualOdometryStereo stereo_odom(param_stereo);

	sprintf(filename, "%06d.png", 0);
	_I0 = cv::imread(dir + "/image_0/" + filename, CV_LOAD_IMAGE_GRAYSCALE);
	_I1 = cv::imread(dir + "/image_1/" + filename, CV_LOAD_IMAGE_GRAYSCALE);

	// image dimensions
	int32_t width = _I0.cols;
	int32_t height = _I0.rows;

	int32_t dims[] =
	{ width, height, width };

	last = _I0;

	for (int i = 0; !_I0.empty();)
	{
		// convert input images to uint8_t buffer
		uint8_t* I0data = (uint8_t*) _I0.data;
		uint8_t* I1data = (uint8_t*) _I1.data;

//		if (mono_odom.process(I0data, dims))
//		{
//			Matrix _motion = mono_odom.getMotion();
//			motion_mono = cv::Mat(_motion.m, _motion.n, CV_32F);
//			for (int i = 0; i < _motion.m; i++)
//			{
//				for (int j = 0; j < _motion.n; j++)
//				{
//					motion_mono.at<float>(i, j) = _motion.val[i][j];
//				}
//			}
//			pose_mono *= motion_mono.inv();
//
//			std::cout << pose_mono << std::endl;
//
//			cv::Mat norm_mat(3, 4, CV_32F);
//			cv::Mat identity = cv::Mat::eye(3, 3, CV_32F);
//			cv::Mat norm_vec = (cv::Mat_<float>(3, 1) << 0, -1, 0);
//			identity.copyTo(norm_mat(cv::Rect(0, 0, 3, 3)));
//			cv::Mat(norm_vec / cam_height).copyTo(
//					norm_mat(cv::Rect(3, 0, 1, 3)));
//			H = K0 * motion_mono.inv()(cv::Rect(0, 0, 4, 3)) * norm_mat.t()
//					* K0.inv();
//
//		}
//		else
//		{
//			std::cout << "Mono failed!" << std::endl;
//		}

		if (stereo_odom.process(I0data, I1data, dims))
		{
			Matrix _motion = stereo_odom.getMotion();
			motion_stereo = cv::Mat(_motion.m, _motion.n, CV_32F);
			for (int i = 0; i < _motion.m; i++)
			{
				for (int j = 0; j < _motion.n; j++)
				{
					motion_stereo.at<float>(i, j) = _motion.val[i][j];
				}
			}
			pose_stereo *= motion_stereo.inv();

			std::cout << pose_stereo << std::endl << std::endl;
		}
		else
		{
			std::cout << "Stereo failed!" << std::endl;
		}

		cv::Mat warped;
		cv::warpPerspective(_I0, warped, H, cv::Size(width, height),
				cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
		cv::imshow(win[2], last - warped);
		last = _I0;

//		cv::Mat display(2 * _I0.rows, _I0.cols, CV_8UC1);
//		_I0.copyTo(display(cv::Rect(0, 0, _I0.cols, _I0.rows)));
//		_I1.copyTo(display(cv::Rect(0, _I1.rows, _I1.cols, _I1.rows)));
//		cv::imshow(win[0], display);

		char key = cv::waitKey(10);
		if (key == 'q')
			break;

		sprintf(filename, "%06d.png", ++i);
		_I0 = cv::imread(dir + "/image_0/" + filename, CV_LOAD_IMAGE_GRAYSCALE);
		_I1 = cv::imread(dir + "/image_1/" + filename, CV_LOAD_IMAGE_GRAYSCALE);

	}

	// output
	std::cout << "Demo complete! Exiting ..." << std::endl;

	// exit
	return 0;
}

