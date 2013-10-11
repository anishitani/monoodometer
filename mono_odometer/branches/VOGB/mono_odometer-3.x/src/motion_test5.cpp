/*
 * motion_test5.cpp
 *
 *  Created on: Jul 19, 2013
 *      Author: nishitani
 */

#include "core.h"
#include "motion_processor.cpp"
#include "image_processor.cpp"
#include "matcher.h"

/**
 *	main()
 */
int main(int argc, char** argv)
{
	LRM::MotionProcessorParameter param;
	LRM::MotionProcessor mot_proc;
//	mot_proc.setting(param);

	std::vector<cv::KeyPoint> query_kpts, train_kpts;
	std::vector<cv::Point2d> query_pts, train_pts;
	std::vector<cv::DMatch> _matches;

	cv::Mat K;
	cv::FileStorage calib_data;
	calib_data.open("/home/nishitani/ros/mono_odometer/calib/camera_left.yaml",
			cv::FileStorage::READ);
	if (calib_data.isOpened())
	{
		calib_data["cameraMatrix"] >> K;
	}

	cv::Mat I1 = cv::imread(
			"/home/nishitani/Dropbox/Photos/Mestrado/dataset/rua/01/0001.jpg",
			CV_LOAD_IMAGE_GRAYSCALE);

	Matcher::parameters match;
	Matcher matcher(match);

	int dim[3] =
	{ I1.cols, I1.rows, I1.cols };

	matcher.pushBack(I1.data, dim, false);

	char c = ' ';

	for (int i = 2; c != 'q'; i++)
	{
		char path[100];
		sprintf(path,
				"/home/nishitani/Dropbox/Photos/Mestrado/dataset/rua/01/%04d.jpg",
				i);
		cv::Mat I2 = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
		matcher.pushBack(I2.data, dim, false);
		matcher.matchFeatures(0);
		matcher.bucketFeatures(5, 50, 50);

		std::vector<Matcher::p_match> matches = matcher.getMatches();
		for (size_t i = 0; i < matches.size(); i++)
		{
			Matcher::p_match m = matches[i];
			train_kpts.push_back(cv::KeyPoint(m.u1p, m.v1p, 1));
			query_kpts.push_back(cv::KeyPoint(m.u1c, m.v1c, 1));
			_matches.push_back(cv::DMatch(i, i, 1));
		}

		mot_proc.matches2points(query_kpts, train_kpts, _matches, query_pts,
				train_pts);
		mot_proc.estimate_motion(train_pts, query_pts, _matches, K);

		cv::Mat out;
		std::vector<char> inliers = mot_proc.getInlierMask();
		LRM::ImageProcessor::draw_displacement(I2, out, query_kpts, train_kpts,
				_matches,inliers);

		cv::imshow("out", out);
		c = cv::waitKey();

		train_kpts.clear();
		query_kpts.clear();
		_matches.clear();
	}

	return 0;
}
