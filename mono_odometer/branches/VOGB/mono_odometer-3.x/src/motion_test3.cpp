/*
 * motion_teste3.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: nishitani
 */

#include "core.h"
#include "motion_processor.cpp"

/**
 *	main()
 */
int main(int argc, char** argv)
{
	LRM::MotionProcessor mot_proc;

	cv::FileStorage K_file(
			"/home/nishitani/Dropbox/Photos/Mestrado/dataset/other/raw/01/cube_pts_calib.yaml",
			cv::FileStorage::READ);
	cv::FileStorage pts_file(
			"/home/nishitani/Dropbox/Photos/Mestrado/dataset/other/raw/01/cube_pts.yaml",
			cv::FileStorage::READ);

	cv::Mat K;
	K_file["cameraMatrix"] >> K;

	cv::Mat x1, x2;
	pts_file["query"] >> x1;
	pts_file["train"] >> x2;

	std::vector<cv::Point2d> X1, X2;

	for (int i = 0; i < x1.cols; i++)
	{
		X1.push_back(cv::Point2d(x1.col(i)));
		X2.push_back(cv::Point2d(x2.col(i)));
	}

	cv::Mat Tc, Tp;
	std::vector<cv::Point2d> X1_norm, X2_norm;

//	mot_proc.feature_point_normalization(X1_norm, X2_norm, Tc, Tp);

	std::cout << "Train Normalized Points: " << std::endl << Tp << std::endl;
	for (int i = 0; i < X2.size(); i++)
	{
		std::cout << X2_norm[i] << std::endl;
	}
	std::cout << "Query Normalized Points: " << std::endl << Tc << std::endl;
	for (int i = 0; i < X1.size(); i++)
	{
		std::cout << X1_norm[i] << std::endl;
	}

	return 0;
}
