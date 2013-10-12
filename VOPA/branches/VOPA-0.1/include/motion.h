/*
 * motion.h
 *
 *  Created on: Sep 26, 2012
 *      Author: nishitani
 */

#ifndef MOTION_H_
#define MOTION_H_

#include "core.h"
#include "frame.h"

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

/*
 *
 */
class MotionEstimator
{
private:
	cv::Mat inliers;

public:
	MotionEstimator();
	virtual ~MotionEstimator();

	void estimate_motion(Frame prev, Frame curr, cv::Mat &P, cv::Mat K = cv::Mat());
	bool feature_point_normalization(std::vector<cv::Point2f> prev_pts,
			std::vector<cv::Point2f> curr_pts,
			std::vector<cv::Point2f> &norm_prev_pts,
			std::vector<cv::Point2f> &norm_curr_pts, cv::Mat &Tp, cv::Mat &Tc);
	cv::Mat compute_F_matrix(std::vector<cv::Point2f> prev_pts,
			std::vector<cv::Point2f> curr_pts);
	cv::Mat compute_Rt(cv::Mat E, cv::Mat K,std::vector<cv::Point2f> prev_pts,
			std::vector<cv::Point2f> curr_pts,
			cv::Mat &R, cv::Mat &t);
	int cheiralityCheck(cv::Mat P2, cv::Mat K, std::vector<cv::Point2f> pp, std::vector<cv::Point2f> pc);
	cv::Mat Rodrigues( cv::Vec3d omega, double theta=-1);
	double triple_product(cv::Vec3d a,cv::Vec3d b,cv::Vec3d c);
	cv::Mat skew(cv::Vec3d a);
};

} /* namespace LRM */
#endif /* MOTION_H_ */
