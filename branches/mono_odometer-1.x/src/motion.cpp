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

	cv::Mat F = compute_F_matrix(norm_prev_pts, norm_curr_pts);

	cv::Mat E = K.t()*Tc.t()*F*Tp*K;

	cv::SVD svd(E);
	svd.w.at<double>(2) = 0;
	E = svd.u*cv::Mat().diag(svd.w)*svd.vt;

	cv::Mat R, t;

	cv::Mat P(compute_Rt(E, K,norm_prev_pts, norm_curr_pts, R, t));

	std::cout << P << std::endl;
	std::cout << "=========END==========" << std::endl;
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
		std::vector<cv::Point2f> curr_pts)
{
	cv::Mat F = cv::findFundamentalMat(prev_pts, curr_pts, cv::FM_RANSAC, 0.01 , 0.99, inliers);

	return F;
}

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
//	std::cout << "detU " << detU << " detV " << detV << std::endl;
//	std::cout << "U1 " << std::endl << U << std::endl << "V1 " << std::endl << V << std::endl;

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

//	std::cout << "detU " << detU << " detV " << detV << std::endl;
//	std::cout << "U2 " << std::endl << U << std::endl << "V2 " << std::endl << V << std::endl;

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

/*
	cv::SVD svd(E);

	cv::Vec3d T;
	cv::Mat Tx, Ra, Rb;
	cv::Mat W = (cv::Mat_<double>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
	cv::Mat Z = (cv::Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 0);

	Tx = (svd.u) * Z * (svd.u.t());
	Ra = (svd.u) * W * (svd.vt);
	Rb = (svd.u) * W.t() * (svd.vt);

	T(0) = Tx.at<double>(2, 1);
	T(1) = Tx.at<double>(0, 2);
	T(2) = Tx.at<double>(1, 0);

	if (cv::determinant(Ra) < 0){
		ROS_DEBUG("Ra determinant is %f",cv::determinant(Ra));
		Ra = -Ra;
	}
	if (cv::determinant(Rb) < 0){
		ROS_DEBUG("Rb determinant is %f",cv::determinant(Rb));
		Rb = -Rb;
	}

	std::vector<cv::Mat> hypothesis(4);

	for (int i = 0; i < 4; i++)
	{
		hypothesis[i] = cv::Mat(3, 4, CV_32F, cv::Scalar(0));
	}

	// Generates the 4 hypotheses of motion transformation
	Ra.copyTo( hypothesis[0]( cv::Range::all(), cv::Range(0,3) ) );
	Ra.copyTo( hypothesis[1]( cv::Range::all(), cv::Range(0,3) ) );
	Rb.copyTo( hypothesis[2]( cv::Range::all(), cv::Range(0,3) ) );
	Rb.copyTo( hypothesis[3]( cv::Range::all(), cv::Range(0,3) ) );

	cv::Mat( T).copyTo(hypothesis[0].col(3));
	cv::Mat(-T).copyTo(hypothesis[1].col(3));
	cv::Mat( T).copyTo(hypothesis[2].col(3));
	cv::Mat(-T).copyTo(hypothesis[3].col(3));
*/

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
		std::cout <<  hypothesis[i] << std::endl;
	}

	return (best==-1) ? cv::Mat().eye(3, 4, CV_32F) : hypothesis[best];
}

int MotionEstimator::cheiralityCheck(cv::Mat P2, cv::Mat K, std::vector<cv::Point2f> pp, std::vector<cv::Point2f> pc)
{
	/*
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
		if(!inliers.at<int>(i,1)) continue;
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
	*/
	cv::Mat R(P2,cv::Range::all(),cv::Range(0,3)),
		    t(P2,cv::Range::all(),cv::Range(3,4)),
		    Tx = (cv::Mat_<float>(3,3) << 0,-t.at<double>(2),t.at<double>(1), t.at<double>(2),0,-t.at<double>(0), -t.at<double>(1),t.at<double>(0),0);

	cv::Mat pph(3,pp.size(),CV_32F,1),pch(3,pp.size(),CV_32F,1);
	pph(cv::Range(0,2),cv::Range::all()) = cv::Mat(pp).reshape(1,pp.size()).t();
	pch(cv::Range(0,2),cv::Range::all()) = cv::Mat(pc).reshape(1,pc.size()).t();

	cv::Mat a(t),b(R*pph),c(Tx*pch);

	int positive_vol=0, positive_depth=0;

	for(uint i=0 ; i<pp.size() ; i++){
//		if(! (int) inliers.data[i]) continue;
		double cond1 = triple_product(cv::Vec3d(a),cv::Vec3d(b.col(i)),cv::Vec3d(c.col(i)));
//		std::cout << "triple product " << cond1 << std::endl;
		if( cond1>0 ) positive_vol++;
		if( cond1<0 ) positive_vol--;

		cv::Mat d( skew(pch.col(i))*t ), e( skew(pch.col(i))*R*pph.col(i) );
		double f = cv::norm(skew(pch.col(i))*t);



		double cond2 = (cv::Mat(-(d.t())*(e)).at<float>(0)/(f*f));
//		std::cout //<< "P " << std::endl << pch.col(i) << std::endl
//				  << "Q " << std::endl << pph.col(i) << std::endl
//				  << "R " << std::endl << R << std::endl
//				  << "T " << std::endl << t << std::endl
//				  << "norm " << std::endl << cv::norm(skew(pch.col(i))*t) << std::endl
//				  << "skew1 " << std::endl << (skew(pch.col(i))*t) << std::endl
//				  << "skew2 " << std::endl << (skew(pch.col(i))*R*pph.col(i)) << std::endl
//				  << "mat " << std::endl << cv::Mat(-(d.t())*(e)) << std::endl
//				  << "prod " << std::endl << cv::Mat(-(d.t())*(e)).at<float>(0) << std::endl;
//
//		std::cout << "alpha1 " << cond2 << std::endl;
//		ros::shutdown();
//		std::cout << "alpha1 " << cond2 << std::endl;
		if( cond2>0 ) positive_depth++;
		if( cond2<0 ) positive_depth--;
	}

//	std::cout << (positive_depth)<< ' ' << (positive_vol) << std::endl;

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

//	ros::shutdown();

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
