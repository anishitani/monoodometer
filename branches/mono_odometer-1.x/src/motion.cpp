/*
 * motion.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: nishitani
 */

//TODO: Attribution of credits to the libviso2 library

#include "motion.h"

namespace LRM
{

MotionEstimator::MotionEstimator()
{
}

MotionEstimator::~MotionEstimator()
{
}

/**
 * estimate_motion:
 * 		Estimates the rotation and translation between two frames.
 *
 * @param prev		Previous analized frame.
 * @param curr		Current analized frame.
 * @param K			Camera matrix.
 */
void MotionEstimator::estimate_motion(Frame prev, Frame curr, cv::Mat &P, cv::Mat K)
{
	std::vector<cv::DMatch> tmpMatchesList(curr.getFeatMatches());
	std::random_shuffle(tmpMatchesList.begin(), tmpMatchesList.end());

	std::vector<cv::Point2f> prev_pts(tmpMatchesList.size());
	std::vector<cv::Point2f> curr_pts(tmpMatchesList.size());

	for (uint i = 0; i < tmpMatchesList.size(); i++)
	{
		prev_pts[i] = prev.getFeatList()[tmpMatchesList[i].trainIdx].pt;
		curr_pts[i] = curr.getFeatList()[tmpMatchesList[i].queryIdx].pt;
	}

	cv::Mat Tp, Tc;
	std::vector<cv::Point2f> norm_prev_pts(tmpMatchesList.size());
	std::vector<cv::Point2f> norm_curr_pts(tmpMatchesList.size());

	if(!feature_point_normalization(prev_pts, curr_pts, norm_prev_pts,
			norm_curr_pts, Tp, Tc))
		return;

	cv::Mat F = compute_F_matrix(norm_prev_pts, norm_curr_pts);

	cv::Mat E = K.t()*Tc.t()*F*Tp*K;

	cv::SVD svd(E);
	svd.w.at<double>(2) = 0;
	E = svd.u*cv::Mat().diag(svd.w)*svd.vt;

	cv::Mat R, t;

	P = compute_Rt(E, K,norm_prev_pts, norm_curr_pts, R, t);

}

bool MotionEstimator::feature_point_normalization(
		std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> curr_pts,
		std::vector<cv::Point2f> &norm_prev_pts,
		std::vector<cv::Point2f> &norm_curr_pts, cv::Mat &Tp, cv::Mat &Tc)
{
	uint matches_size = prev_pts.size();
	cv::Point2f cent_prev(0, 0), cent_curr(0, 0);
	for (uint i = 0; i < matches_size; i++)
	{
		cent_prev += prev_pts[i];
		cent_curr += curr_pts[i];
	}
	cent_prev.x /= (double) matches_size;
	cent_prev.y /= (double) matches_size;
	cent_curr.x /= (double) matches_size;
	cent_curr.y /= (double) matches_size;
	for (uint i = 0; i < matches_size; i++)
	{
		norm_prev_pts[i] = prev_pts[i] - cent_prev;
		norm_curr_pts[i] = curr_pts[i] - cent_curr;
	}

	// scale features such that mean distance from origin is sqrt(2)
	double sp = 0, sc = 0;
	for (uint i = 0; i < matches_size; i++)
	{
		sp += cv::norm(norm_prev_pts[i]);
		sc += cv::norm(norm_curr_pts[i]);
	}
	if (fabs(sp) < 1e-10 || fabs(sc) < 1e-10)
		return false;
	sp = sqrt(2.0) * (double) matches_size / sp;
	sc = sqrt(2.0) * (double) matches_size / sc;
	for (uint i = 0; i < matches_size; i++)
	{
		norm_prev_pts[i] *= sp;
		norm_curr_pts[i] *= sc;
	}

	// compute corresponding transformation matrices
	Tp = cv::Mat(cv::Matx33d(sp, 0, -sp * cent_prev.x, 0, sp, -sp * cent_prev.y, 0, 0, 1));
	Tc = cv::Mat(cv::Matx33d(sc, 0, -sc * cent_curr.x, 0, sc, -sc * cent_curr.y, 0, 0, 1));

	return true;
}


cv::Mat MotionEstimator::compute_F_matrix(std::vector<cv::Point2f> prev_pts,
		std::vector<cv::Point2f> curr_pts)
{
	cv::Mat F = cv::findFundamentalMat(prev_pts, curr_pts, cv::FM_RANSAC, 0.1 , 0.99, inliers);

	std::cout << F << std::endl;

	return F;
}

/*
cv::Mat MotionEstimator::compute_F_matrix (std::vector<cv::Point2f> prev_pts,
		std::vector<cv::Point2f> curr_pts) {

  // number of active p_matched
  uint N = prev_pts.size();

  // create constraint matrix A
  cv::Mat A(N,9,CV_64F);

  for (uint i=0; i<N; i++) {
//    Matcher::p_match m = p_matched[active[i]];
    A.at<float>(i,0) = curr_pts[i].x*prev_pts[i].x;
    A.at<float>(i,1) = curr_pts[i].x*prev_pts[i].y;
    A.at<float>(i,2) = curr_pts[i].x;
    A.at<float>(i,3) = curr_pts[i].y*prev_pts[i].x;
    A.at<float>(i,4) = curr_pts[i].y*prev_pts[i].y;
    A.at<float>(i,5) = curr_pts[i].y;
    A.at<float>(i,6) = prev_pts[i].x;
    A.at<float>(i,7) = prev_pts[i].y;
    A.at<float>(i,8) = 1;
  }

  // compute singular value decomposition of A
//  Mat U,W,V;
  cv::SVD svd(A);

  // extract fundamental matrix from the column of V corresponding to the smallest singular value
  cv::Mat F = cv::Mat(svd.vt.row(8));// Matrix::reshape(V.getMat(0,8,8,8),3,3);
  F = F.reshape(0,3);

  // enforce rank 2
  svd(F);
  svd.w.at<double>(3) = 0;

  return svd.u*cv::Mat::diag(svd.w)*svd.vt;
}
*/

cv::Mat MotionEstimator::compute_Rt(cv::Mat E, cv::Mat K,std::vector<cv::Point2f> prev_pts,
		std::vector<cv::Point2f> curr_pts,
		cv::Mat &R, cv::Mat &t)
{

	cv::SVD positive(E);
	cv::SVD negative(-E);

	cv::Mat Rzp = (cv::Mat_<double>(3, 3) << 0, -1, 0,  1, 0, 0, 0, 0, 1);
	cv::Mat Rzn = (cv::Mat_<double>(3, 3) << 0,  1, 0, -1, 0, 0, 0, 0, 1);
	cv::Mat W   = (cv::Mat_<double>(3, 3) << 1,0,0, 0,1,0, 0,0,0);

	cv::Mat U,V,T,Tx,Rot;

	std::vector<cv::Mat> hypothesis(4);
	for (int i = 0; i < 4; i++)
		hypothesis[i] = cv::Mat(3, 4, CV_32F, cv::Scalar(0));

	//============ +E =================
	U = positive.u;
	V = positive.vt.t();

	double detU = cv::determinant(U);
	double detV = cv::determinant(V);

	if(detU<0 && detV<0){
		U  = -U;
		V = -V;
	}
	else if(detU<0 && detV>0){
		U = -U * Rodrigues(cv::Vec3d(0,0,1),CV_PI) * Rzp;
		V =  V * Rzp;
	}
	else if(detU>0 && detV<0){
		U =  U * Rodrigues(cv::Vec3d(0,0,1),CV_PI) * Rzp;
		V = -V * Rzp;
	}

	Tx  = U * (Rzp*W) * U.t();
	Rot = U * Rzp.t() * V.t();

	T = (cv::Mat_<double>(3,1) << Tx.at<double>(2, 1),Tx.at<double>(0, 2),Tx.at<double>(1, 0));
	T.copyTo(hypothesis[0].col(3));
	Rot.copyTo( hypothesis[0](cv::Range::all(), cv::Range(0,3)) );

	Tx  = U * (Rzn*W) * U.t();
	Rot = U * Rzn.t() * V.t();

	T = (cv::Mat_<double>(3,1) << Tx.at<double>(2, 1),Tx.at<double>(0, 2),Tx.at<double>(1, 0));
	T.copyTo(hypothesis[1].col(3));
	Rot.copyTo( hypothesis[1](cv::Range::all(), cv::Range(0,3)) );
	//=================================

	//============ -E =================
	U = negative.u;
	V = negative.vt.t();

	detU = cv::determinant(U);
	detV = cv::determinant(V);

	if(detU<0 && detV<0){
		U = -U;
		V = -V;
	}
	else if(detU<0 && detV>0){
		U = -U * Rodrigues(cv::Vec3d(0,0,1),CV_PI) * Rzp;
		V =  V * Rzp;
	}
	else if(detU>0 && detV<0){
		U =  U * Rodrigues(cv::Vec3d(0,0,1),CV_PI) * Rzp;
		V = -V * Rzp;
	}

	Tx  = U * (Rzp*W) * U.t();
	Rot = U * Rzp.t() * V.t();

	T = (cv::Mat_<double>(3,1) << Tx.at<double>(2, 1),Tx.at<double>(0, 2),Tx.at<double>(1, 0));
	T.copyTo(hypothesis[2].col(3));
	Rot.copyTo( hypothesis[2](cv::Range::all(), cv::Range(0,3)) );

	Tx  = U * (Rzn*W) * U.t();
	Rot = U * Rzn.t() * V.t();

	T = (cv::Mat_<double>(3,1) << Tx.at<double>(2, 1),Tx.at<double>(0, 2),Tx.at<double>(1, 0));
	T.copyTo(hypothesis[3].col(3));
	Rot.copyTo( hypothesis[3](cv::Range::all(), cv::Range(0,3)) );
	//=================================

	int inliers = 0, best = -1;
 	for (int i = 0; i < 4; i++)
	{
		/*
		 * pp: Previous image point
		 * pc: Current image point
		 */

		int positive_check = cheiralityCheck(hypothesis[i], K, prev_pts, curr_pts);
		if(positive_check > inliers){
			inliers = positive_check;
			best = i;
		}
	}

	return (best==-1) ? cv::Mat().eye(3, 4, CV_32F) : hypothesis[best];
}

int MotionEstimator::cheiralityCheck(cv::Mat P2, cv::Mat K, std::vector<cv::Point2f> pp, std::vector<cv::Point2f> pc)
{
	cv::Mat R(P2,cv::Range::all(),cv::Range(0,3)),
		    t(P2,cv::Range::all(),cv::Range(3,4)),
		    Tx = (cv::Mat_<float>(3,3) << 0,-t.at<double>(2),t.at<double>(1), t.at<double>(2),0,-t.at<double>(0), -t.at<double>(1),t.at<double>(0),0);

	cv::Mat pph(3,pp.size(),CV_32F,1),pch(3,pp.size(),CV_32F,1);
	pph(cv::Range(0,2),cv::Range::all()) = cv::Mat(pp).reshape(1,pp.size()).t();
	pch(cv::Range(0,2),cv::Range::all()) = cv::Mat(pc).reshape(1,pc.size()).t();

	cv::Mat a(t),b(R*pph),c(Tx*pch);

	int positive_vol=0, positive_depth=0;

	for(uint i=0 ; i<pp.size() ; i++){
		double cond1 = triple_product(cv::Vec3d(a),cv::Vec3d(b.col(i)),cv::Vec3d(c.col(i)));
		if( cond1>0 ) positive_vol++;
		if( cond1<0 ) positive_vol--;

		cv::Mat d( skew(pch.col(i))*t ), e( skew(pch.col(i))*R*pph.col(i) );
		double f = cv::norm(skew(pch.col(i))*t);

		double cond2 = (cv::Mat(-(d.t())*(e)).at<float>(0)/(f*f));
		if( cond2>0 ) positive_depth++;
		if( cond2<0 ) positive_depth--;
	}

	return (positive_depth+positive_vol);
}


cv::Mat MotionEstimator::Rodrigues( cv::Vec3d omega, double theta){
	double norm_omega = cv::norm(omega);

	if (theta == -1){
		theta = norm_omega;
		omega = omega/norm_omega;
		norm_omega = cv::norm(omega);
	}

	cv::Mat omega_hat = (cv::Mat_<double>(3, 3) << 0,-omega(2),omega(1),omega(2),0,-omega(0),-omega(1),omega(0),0);

	cv::Vec3d unit(1.,1.,1.);

	return ( cv::Mat().diag(cv::Mat(unit))+(omega_hat/norm_omega)*cv::sin(norm_omega*theta)+((omega_hat*omega_hat)/(norm_omega*norm_omega))*(1-cv::cos(norm_omega*theta)) );

}

double MotionEstimator::triple_product(cv::Vec3d a,cv::Vec3d b,cv::Vec3d c){
	return c(0)*(a(1)*b(2)-a(2)*b(1)) + c(1)*(a(2)*b(0) - b(2)*a(0)) + c(2)*(a(0)*b(1) - b(0)*a(1));
}

cv::Mat MotionEstimator::skew(cv::Vec3d a){
	return (cv::Mat_<float>(3,3) << 0,-a(2),a(1), a(2),0,-a(0), -a(1),a(0),0);
}

} /* namespace LRM */
