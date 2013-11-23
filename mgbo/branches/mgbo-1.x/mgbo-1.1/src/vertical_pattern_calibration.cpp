/*
 * vertical_pattern_calibration.cpp
 *
 *  Created on: Oct 8, 2013
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

#include "camera_tool.hpp"

int main(int argc, char** argv)
{

	if (argc < 2)
	{
		printf("Usage: ./odometer <image_dataset_folder>");
		return 1;
	}

	// sequence directory
	std::string dir = argv[1];

	cv::Mat K =
			(cv::Mat_<float>(3, 3) << 619.970819, 0, 311.431502, 0, 619.64086, 215.838431, 0, 0, 1);
	cv::Mat D =
			(cv::Mat_<float>(1, 5) << -0.407065, 0.181976, -1e-06, 0.004683, 0);

	cv::Mat H(3, 3, CV_32F);
	cv::Mat H0 = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

	cv::Mat ChessRot = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 0, 1, 0, -1, 0);

	std::string image_win("image");

	// loop through all frames
	for (int32_t i = 1;; i++)
	{
		// catch image read/write errors here
		try
		{
			char path[256];

			sprintf(path, "%s/%d.jpeg",
					*(dir.end() - 1) != '/' ?
							std::string(dir + "/").c_str() : dir.c_str(), i);

			// load input image
			cv::Mat _img = cv::imread(path);
			cv::Mat img;

			if (_img.empty())
				break;

			cv::undistort(_img, img, K, D);

			cv::Mat corners;
			cv::Mat chess3d;
			bool found = cv::findChessboardCorners(img, cv::Size(9, 6),
					corners);
			cv::flip(corners, corners, 0);
			cv::drawChessboardCorners(img, cv::Size(9, 6), corners, found);
			chess3d = CameraTool::chessboard3D_points(6, 9, 29);
			chess3d.col(1) += 11;
			chess3d.convertTo(chess3d, CV_32F);
			chess3d = (ChessRot * chess3d.t()).t();

			std::cout << corners << std::endl;
			std::cout << chess3d << std::endl;

			cv::Mat pose = CameraTool::cameraPoseFromMatches(corners, chess3d,
					K);
			pose.convertTo(pose,CV_32F);
			pose(cv::Rect(0, 0, 1, 3)).copyTo(H.col(0));
			pose(cv::Rect(1, 0, 1, 3)).copyTo(H.col(1));
			pose(cv::Rect(3, 0, 1, 3)).copyTo(H.col(2));

//			cv::Mat C = (cv::Mat_<float>(3, 1) << 0, -200, -8);
//			cv::Mat(pose(cv::Rect(0, 0, 3, 3)) * C).copyTo(H.col(2));

			std::cout << pose << std::endl;
			std::cout << H << std::endl;

			cv::Mat warped;
			cv::warpPerspective(img, warped, K * H0 * H, cv::Size(640, 480),
					cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
			cv::imshow("warped", warped);

			cv::imshow(image_win, img);
			char key = cv::waitKey(0);
			if (key == 'q')
				break;

			// catch image read errors here
		} catch (...)
		{
			std::cerr << "ERROR: Couldn't read input files!" << std::endl;
			return 1;
		}
	}

// output
	std::cout << "Demo complete! Exiting ..." << std::endl;

// exit
	return 0;
}

