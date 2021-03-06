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
	// TODO Auto-generated constructor stub

}

MotionEstimator::~MotionEstimator()
{
	// TODO Auto-generated destructor stub
}

/**
 * estimate_motion:
 * 		Estimates the rotation and translation between two frames.
 *
 * @param prev		Previous analized frame.
 * @param curr		Current analized frame.
 * @param K			Camera matrix.
 */
void MotionEstimator::estimate_motion(Frame prev, Frame curr, cv::Mat K)
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

	cv::Mat outliers;


	cv::Mat F = compute_F_matrix(norm_prev_pts, norm_curr_pts, outliers);

	cv::Mat E = K.t()*Tc.t()*F*Tp*K;

	cv::SVD svd(E);
	svd.w.at<double>(2) = 0;
	E = svd.u*cv::Mat().diag(svd.w)*svd.vt;

	cv::Mat R, t;

	compute_Rt(E, K,norm_prev_pts, norm_curr_pts, outliers, R, t);

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
	cent_prev *= 1/(double) matches_size;
	cent_curr *= 1/(double) matches_size;
	for (uint i = 0; i < matches_size; i++)
	{
		norm_prev_pts[i] = prev_pts[i] - cent_prev;
		norm_curr_pts[i] = curr_pts[i] - cent_curr;
	}

	// scale features such that mean distance from origin is sqrt(2)
	double sp = 0, sc = 0;
	for (uint i = 0; i < matches_size; i++)
	{
		sp += cv::norm(prev_pts[i]);
		sc += cv::norm(curr_pts[i]);
	}
	if (fabs(sp) < 1e-10 || fabs(sc) < 1e-10)
		return false;
	sp = sqrt(2.0) * (double) matches_size / sp;
	sc = sqrt(2.0) * (double) matches_size / sc;
	for (uint i = 0; i < matches_size; i++)
	{
		norm_prev_pts[i] = prev_pts[i] * sp;
		norm_curr_pts[i] = curr_pts[i] * sc;
	}

	// compute corresponding transformation matrices
	Tp = cv::Mat(cv::Matx33d(sp, 0, -sp * cent_prev.x, 0, sp, -sp * cent_prev.y, 0, 0, 1));
	Tc = cv::Mat(cv::Matx33d(sc, 0, -sc * cent_curr.x, 0, sc, -sc * cent_curr.y, 0, 0, 1));

	return true;
}

cv::Mat MotionEstimator::compute_F_matrix(std::vector<cv::Point2f> prev_pts,
		std::vector<cv::Point2f> curr_pts, cv::Mat &outliers)
{
	cv::Mat F = cv::findFundamentalMat(prev_pts, curr_pts, cv::FM_RANSAC, 3. , 0.999, outliers);
	cv::SVD svd(F);

	if(curr_pts.size()==(uint)outliers.cols){
		std::cout << "Outliers: " << std::endl << outliers << std::endl << std::endl;
		ros::shutdown();
	}

	return svd.u*cv::Mat().diag(svd.w)*svd.vt ;
}

cv::Mat MotionEstimator::compute_Rt(cv::Mat E, cv::Mat K,std::vector<cv::Point2f> prev_pts,
		std::vector<cv::Point2f> curr_pts,
		std::vector<int> outliers, cv::Mat &R, cv::Mat &t)
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
		U = -U * Rodrigues(W.diag(),CV_PI) * Rzp;
		V =  V * Rzp;
	}
	else if(detU>0 && detV<0){
		U =  U * Rodrigues(W.diag(),CV_PI) * Rzp;
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
		U = -U * Rodrigues(W.diag(),CV_PI) * Rzp;
		V =  V * Rzp;
	}
	else if(detU>0 && detV<0){
		U =  U * Rodrigues(W.diag(),CV_PI) * Rzp;
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
		std::cout << "hypotesis(" << i << ") inliers: " << positive_check << std::endl;
//		std::cout <<  hypothesis[i] << std::endl;
	}
	std::cout << "=========END==========" << std::endl;

	return (best==-1) ? cv::Mat().eye(3, 4, CV_32F) : hypothesis[best];
}

int MotionEstimator::cheiralityCheck(cv::Mat P2, cv::Mat K, std::vector<cv::Point2f> pp, std::vector<cv::Point2f> pc)
{
	//TODO: Reduce the number of checked points, otherwise the reason
	//to use the ground as a motion estimator is invalid.
	cv::Mat P1 = cv::Mat().eye(3, 4, CV_32F);

	K.convertTo(K,CV_32F);

	P1 = K*P1;
	P2 = K*P2;

	cv::Mat J(4, 4, CV_32F);
	cv::Mat X(4,pp.size(),CV_32F);
	int num = 0;

	for(uint i=0 ; i<pp.size() ; i++){
		cv::Mat L1( pc[i].x*P2.row(2)-P2.row(0) );
		cv::Mat L2( pc[i].y*P2.row(2)-P2.row(1) );
		cv::Mat L3( pp[i].x*P1.row(2)-P1.row(0) );
		cv::Mat L4( pp[i].y*P1.row(2)-P1.row(1) );
		L1.copyTo( J.row(0) );
		L2.copyTo( J.row(1) );
		L3.copyTo( J.row(2) );
		L4.copyTo( J.row(3) );

		cv::SVD svd(J);
		cv::Mat(svd.vt.t(),cv::Range::all(),cv::Range(3,4)).copyTo(X.col(i));
	}

	// compute inliers
	cv::Mat AX1(P1*X);
	cv::Mat BX1(P2*X);

	for (int i=0; i<X.cols; i++)
		if ( (AX1.at<double>(0,2)*X.at<double>(0,3) > 0) && (BX1.at<double>(0,2)*X.at<double>(0,3) > 0) )
			num++;

	// return number of inliers
	return num;
}


cv::Mat MotionEstimator::Rodrigues( cv::Vec3d omega, double theta){
	double norm_omega = cv::norm(omega);

	if (theta == -1){
		theta = norm_omega;
		omega = omega/norm_omega;
		norm_omega = cv::norm(omega);
	}

	cv::Mat omega_hat = (cv::Mat_<double>(3, 3) << 0,-omega(3),omega(2),omega(3),0,-omega(1),-omega(2),omega(1),0);

	cv::Vec3d unit(1.,1.,1.);
	return ( cv::Mat().diag(cv::Mat(unit))+(omega_hat/norm_omega)*cv::sin(norm_omega*theta)+((omega_hat*omega_hat)/norm_omega*norm_omega)*(1-cv::cos(norm_omega*theta)) );

}

} /* namespace LRM */
