/*
 * motion.h
 *
 *  Created on: Sep 26, 2012
 *      Author: nishitani
 */

#ifndef MOTION_H_
#define MOTION_H_

#include "core.h"

typedef double Matches[][3];
typedef double Matches_5[5][3];
typedef double Ematrix[3][3];

/**
 * compute_E_matrices:
 *
 * @param q
 * @param qp
 * @param Ematrices
 * @param nroots
 * @param optimized
 */
void compute_E_matrices(Matches q, Matches qp, Ematrix Ematrices[10],
		int &nroots, bool optimized);

namespace LRM
{

class MotionEstimator
{
private:
	std::vector<char> inliers;
	std::vector<cv::Point2d> train_pts, query_pts;

public:
	MotionEstimator();
	virtual ~MotionEstimator();

	void matches2points(const std::vector<cv::KeyPoint>& query,
			const std::vector<cv::KeyPoint>& train,
			const std::vector<cv::DMatch>& matches,
			std::vector<cv::Point2d> &query_pts,
			std::vector<cv::Point2d> &train_pts);
	void estimate_motion(std::vector<cv::Point2d> train_pts,
			std::vector<cv::Point2d> query_pts, std::vector<cv::DMatch> matches,
			cv::Mat K, cv::Mat &P);
	bool feature_point_normalization(std::vector<cv::Point2d> query_pts,
			std::vector<cv::Point2d> train_pts,
			std::vector<cv::Point2d> &norm_query_pts,
			std::vector<cv::Point2d> &norm_train_pts, cv::Mat &Tc, cv::Mat &Tp);
	cv::Mat compute_F_matrix(std::vector<cv::Point2d> train_pts,
			std::vector<cv::Point2d> query_pts);
	std::vector<cv::Mat> compute_Rt(cv::Mat E, cv::Mat K);
	int triangulateCheck(std::vector<cv::Point2d> train_pts,
			std::vector<cv::Point2d> query_pts, cv::Mat &K, cv::Mat P,
			std::vector<char> mask = std::vector<char>());
	int cheiralityCheck(cv::Mat P2, cv::Mat K, std::vector<cv::Point2d> pp,
			std::vector<cv::Point2d> pc);
	cv::Mat Rodrigues(cv::Vec3d omega, double theta = -1);
	double triple_product(cv::Vec3d a, cv::Vec3d b, cv::Vec3d c);
	cv::Mat skew(cv::Vec3d a);

	//gets
	std::vector<char> getInliers(){ return inliers; }
};

} /* namespace LRM */
#endif /* MOTION_H_ */
