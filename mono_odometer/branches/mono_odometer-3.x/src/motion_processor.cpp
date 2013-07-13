//TODO: Attribution of credits to the libviso2 library
#include "motion_processor.h"

namespace LRM
{
///////////////////////////////////////////////////////////////////////
/**				Motion Processor Parameter Class					**/
///////////////////////////////////////////////////////////////////////
int MotionProcessorParameter::parse(ros::NodeHandle nh)
{
	double value_double;

	nh.param<double>("RANSAC_EPIPOLAR_DIST", value_double, 3.0);
	parameter["RANSAC_EPIPOLAR_DIST"] = value_double;

	nh.param<double>("RANSAC_CONFIDENCE", value_double, 0.95);
	parameter["RANSAC_CONFIDENCE"] = value_double;

	ROS_DEBUG("RANSAC_EPIPOLAR_DIST = %f",
			boost::any_cast<double>(parameter["RANSAC_EPIPOLAR_DIST"]));
	ROS_DEBUG("RANSAC_CONFIDENCE = %f",
			boost::any_cast<double>(parameter["RANSAC_CONFIDENCE"]));

	return 0;
}

///////////////////////////////////////////////////////////////////////
/**						Motion Processor Class						**/
///////////////////////////////////////////////////////////////////////
MotionProcessor::MotionProcessor()
{
	confidence = 0.95;
	epipolar_dist = 3.0;
}

MotionProcessor::~MotionProcessor()
{
}

int MotionProcessor::setting(MotionProcessorParameter param)
{
	int MP_ERR_CODE = 0; //Error Code

	epipolar_dist = param.getParameterByName<double>("RANSAC_EPIPOLAR_DIST");
	confidence = param.getParameterByName<double>("RANSAC_CONFIDENCE");

	return MP_ERR_CODE;
}

/**
 * matches2points:
 * 	Tranform the keypoints matched in a matched list of points.
 *
 * @param query			Current set of keypoints.
 * @param train			Last set of keypoints.
 * @param matches		Matches between the current and last keypoints.
 * @param query_pts		Current set of 2D points.
 * @param train_pts		Last set of 2D points.
 */
void MotionProcessor::matches2points(const std::vector<cv::KeyPoint>& query,
		const std::vector<cv::KeyPoint>& train,
		const std::vector<cv::DMatch>& matches,
		std::vector<cv::Point2d> &query_pts,
		std::vector<cv::Point2d> &train_pts)
{
	train_pts.clear();
	query_pts.clear();

	for (int i = 0; i < (int) matches.size(); i++)
	{
		const cv::DMatch & dmatch = matches[i];

		query_pts.push_back(query[dmatch.queryIdx].pt);
		train_pts.push_back(train[dmatch.trainIdx].pt);
	}
}

/**
 * @brief Estimates the rotation and translation between two frames.
 *
 * @param train_pts		Previous image feature points.
 * @param query_pts		Current image feature points.
 * @param matches		Matches previous and current feature points indexers.
 * @param K				Intrinsic parameters matrix.
 */
void MotionProcessor::estimate_motion(std::vector<cv::Point2d> train_pts,
		std::vector<cv::Point2d> query_pts, std::vector<cv::DMatch> matches,
		cv::Mat K)
{
	//Normalization transformations
	cv::Mat Tp, Tc;

	//Normalized vectors
	std::vector<cv::Point2d> norm_train_pts(train_pts.size());
	std::vector<cv::Point2d> norm_query_pts(query_pts.size());

	//Fundamental matrix
	cv::Mat F;

	if (!feature_point_normalization(query_pts, train_pts, norm_query_pts,
			norm_train_pts, Tc, Tp))
	{
		P = cv::Mat().eye(4, 4, CV_64F);
		return;
	}

	F = compute_F_matrix(norm_train_pts, norm_query_pts);
	if (!cv::countNonZero(F))
	{
		P = cv::Mat::eye(4, 4, CV_64F);
		return;
	}

	cv::SVD svd(F);
	svd.w.at<double>(0) = 1;
	svd.w.at<double>(1) = 1;
	svd.w.at<double>(2) = 0;

	F = svd.u * cv::Mat().diag(svd.w) * svd.vt;

	cv::Mat E = K.t() * Tc.t() * F * Tp * K;

	cv::Mat R, t;

	std::vector<cv::Mat> P_vec;
	P_vec = compute_Rt(E, K);

	int max_inliers = 0;
	int pos = 0;

	std::vector<std::vector<char> > mask_vec;

	for (size_t i = 0; i < P_vec.size(); i++)
	{
		std::vector<char> _mask(inliers);
		int num_inliers = triangulateCheck(train_pts, query_pts, K, P_vec[i],
				_mask);
		if (max_inliers < num_inliers)
		{
			max_inliers = num_inliers;
			pos = i;
		}
		mask_vec.push_back(_mask);
	}

	P = P_vec[pos];

	inliers = mask_vec[pos];

}

bool MotionProcessor::feature_point_normalization(
		std::vector<cv::Point2d> query_pts, std::vector<cv::Point2d> train_pts,
		std::vector<cv::Point2d> &norm_query_pts,
		std::vector<cv::Point2d> &norm_train_pts, cv::Mat &Tc, cv::Mat &Tp)
{
	uint matches_size = train_pts.size();
	cv::Point2d cent_train(0, 0), cent_query(0, 0);
	for (uint i = 0; i < matches_size; i++)
	{
		cent_train += train_pts[i];
		cent_query += query_pts[i];
	}
	cent_train.x /= (double) matches_size;
	cent_train.y /= (double) matches_size;
	cent_query.x /= (double) matches_size;
	cent_query.y /= (double) matches_size;
	for (uint i = 0; i < matches_size; i++)
	{
		norm_train_pts[i] = train_pts[i] - cent_train;
		norm_query_pts[i] = query_pts[i] - cent_query;
	}

	// scale features such that mean distance from origin is sqrt(2)
	double sp = 0, sc = 0;
	for (uint i = 0; i < matches_size; i++)
	{
		sp += cv::norm(norm_train_pts[i]);
		sc += cv::norm(norm_query_pts[i]);
	}
	if (fabs(sp) < 1e-10 || fabs(sc) < 1e-10)
		return false;
	sp = sqrt(2.0) * (double) matches_size / sp;
	sc = sqrt(2.0) * (double) matches_size / sc;
	for (uint i = 0; i < matches_size; i++)
	{
		norm_train_pts[i] *= sp;
		norm_query_pts[i] *= sc;
	}

	// compute corresponding transformation matrices
	Tp = cv::Mat(
			cv::Matx33d(sp, 0, -sp * cent_train.x, 0, sp, -sp * cent_train.y, 0,
					0, 1));
	Tc = cv::Mat(
			cv::Matx33d(sc, 0, -sc * cent_query.x, 0, sc, -sc * cent_query.y, 0,
					0, 1));

	return true;
}

cv::Mat MotionProcessor::compute_F_matrix(std::vector<cv::Point2d> train_pts,
		std::vector<cv::Point2d> query_pts)
{
	cv::Mat mask;
	cv::Mat F = cv::findFundamentalMat(train_pts, query_pts, cv::FM_RANSAC,
			epipolar_dist, confidence, mask);

	inliers = std::vector<char>(mask);

	//	std::cout << F << std::endl;

	return F;
}

std::vector<cv::Mat> MotionProcessor::compute_Rt(cv::Mat E, cv::Mat K)
{

	// hartley matrices
	double w[3][3] =
	{
	{ 0, -1, 0 },
	{ +1, 0, 0 },
	{ 0, 0, 1 } };
	double z[3][3] =
	{
	{ 0, +1, 0 },
	{ -1, 0, 0 },
	{ 0, 0, 0 } };
	cv::Mat W = cv::Mat(3, 3, CV_64F, w);
	cv::Mat Z = cv::Mat(3, 3, CV_64F, z);

	// extract T,R1,R2 (8 solutions)
	cv::SVD svd;
	svd(E);

	cv::Mat tp = svd.u.col(2);
	cv::Mat tn = -svd.u.col(2);
	cv::Mat Ra = svd.u * W * svd.vt;
	cv::Mat Rb = svd.u * W.t() * svd.vt;

	// assure determinant to be positive
	if (cv::determinant(Ra) < 0)
		Ra = -Ra;
	if (cv::determinant(Rb) < 0)
		Rb = -Rb;

	// create vector containing all 4 solutions
	std::vector<cv::Mat> P; //(4,Mat::eye(4,4,CV_64F));

	cv::Mat P1 = cv::Mat::eye(4, 4, CV_64F);
	Ra.copyTo(P1(cv::Range(0, 3), cv::Range(0, 3)));
	tp.copyTo(P1(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P1);

	cv::Mat P2 = cv::Mat::eye(4, 4, CV_64F);
	Ra.copyTo(P2(cv::Range(0, 3), cv::Range(0, 3)));
	tn.copyTo(P2(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P2);

	cv::Mat P3 = cv::Mat::eye(4, 4, CV_64F);
	Rb.copyTo(P3(cv::Range(0, 3), cv::Range(0, 3)));
	tp.copyTo(P3(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P3);

	cv::Mat P4 = cv::Mat::eye(4, 4, CV_64F);
	Rb.copyTo(P4(cv::Range(0, 3), cv::Range(0, 3)));
	tn.copyTo(P4(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P4);

	return P;
}

int MotionProcessor::triangulateCheck(std::vector<cv::Point2d> train_pts,
		std::vector<cv::Point2d> query_pts, cv::Mat &K, cv::Mat P,
		std::vector<char> mask)
{
	// init 3d point matrix
	cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
	cv::Mat P1, P2;
	cv::Mat X_vec;
	cv::SVD svd;

	int num_inliers = 0;

	// projection matrices
	P1 = K * cv::Mat::eye(3, 4, CV_64F);
	P2 = K * P(cv::Range(0, 3), cv::Range::all());

	// triangulation via orthogonal regression
	for (int i = 0; i < (int) mask.size(); i++)
	{
		if (!(int) mask[i])
			continue;

		/* ********************************** */
		/*  Set the homogeneous equation AX=0 */
		/* ********************************** */
		A.row(0) += (query_pts[i].x * P1.row(2).t() - P1.row(0).t()).t();
		A.row(1) += (query_pts[i].y * P1.row(2).t() - P1.row(1).t()).t();
		A.row(2) += (train_pts[i].x * P2.row(2).t() - P2.row(0).t()).t();
		A.row(3) += (train_pts[i].y * P2.row(2).t() - P2.row(1).t()).t();
		/* ********************************** */

		svd(A);

		cv::Mat X(svd.vt.row(3).t());

		/* ******************** */
		/* Checa a profundidade */
		/* ******************** */
		cv::Mat x1 = P1 * X, x2 = P2 * X;
//		cv::Mat M1(P1, cv::Rect(0, 0, 3, 3)), M2(P2, cv::Rect(0, 0, 3, 3));
//		int sign1 = cv::determinant(M1) > 0 ? 1 : -1, sign2 =
//				cv::determinant(M2) > 0 ? 1 : -1;
//
//		double depth1 = (sign1 * x1.at<double>(2, 0))
//				/ (X.at<double>(0, 3) * norm(M1.col(2))), depth2 = (sign2
//				* x2.at<double>(2, 0)) / (X.at<double>(0, 3) * norm(M2.col(2)));
//		if (depth1 > 0 && depth2 > 0)
//		{
//			num_inliers++;
//		}
//		else
//		{
//			mask[i] = (char) 0;
//		}
		if (x1.at<double>(2, 0) * X.at<double>(3, 0) > 0
				&& x2.at<double>(2, 0) * X.at<double>(3, 0) > 0)
			num_inliers++;
		/**************************/

		X /= X.at<double>(3, 0);

		X_vec.push_back(X);

		A = cv::Mat::zeros(4, 4, CV_64F);
	}

	// return number of inliers
	return num_inliers;
}

int MotionProcessor::cheiralityCheck(cv::Mat P2, cv::Mat K,
		std::vector<cv::Point2d> pp, std::vector<cv::Point2d> pc)
{
	cv::Mat R(P2, cv::Range::all(), cv::Range(0, 3)), t(P2, cv::Range::all(),
			cv::Range(3, 4)), Tx = (cv::Mat_<float>(3, 3) << 0, -t.at<double>(
			2), t.at<double>(1), t.at<double>(2), 0, -t.at<double>(0), -t.at<
			double>(1), t.at<double>(0), 0);

	cv::Mat pph(3, pp.size(), CV_32F, 1), pch(3, pp.size(), CV_32F, 1);
	pph(cv::Range(0, 2), cv::Range::all()) =
			cv::Mat(pp).reshape(1, pp.size()).t();
	pch(cv::Range(0, 2), cv::Range::all()) =
			cv::Mat(pc).reshape(1, pc.size()).t();

	cv::Mat a(t), b(R * pph), c(Tx * pch);

	int positive_vol = 0, positive_depth = 0;

	for (uint i = 0; i < pp.size(); i++)
	{
		double cond1 = triple_product(cv::Vec3d(a), cv::Vec3d(b.col(i)),
				cv::Vec3d(c.col(i)));
		if (cond1 > 0)
			positive_vol++;
		if (cond1 < 0)
			positive_vol--;

		cv::Mat d(skew(pch.col(i)) * t), e(skew(pch.col(i)) * R * pph.col(i));
		double f = cv::norm(skew(pch.col(i)) * t);

		double cond2 = (cv::Mat(-(d.t()) * (e)).at<float>(0) / (f * f));
		if (cond2 > 0)
			positive_depth++;
		if (cond2 < 0)
			positive_depth--;
	}

	return (positive_depth + positive_vol);
}

int MotionProcessor::noMotion()
{
	P = cv::Mat::eye(4,4,CV_64F);
	return 0;
}

cv::Mat MotionProcessor::Rodrigues(cv::Vec3d omega, double theta)
{
	double norm_omega = cv::norm(omega);

	if (theta == -1)
	{
		theta = norm_omega;
		omega = omega / norm_omega;
		norm_omega = cv::norm(omega);
	}

	cv::Mat omega_hat =
			(cv::Mat_<double>(3, 3) << 0, -omega(2), omega(1), omega(2), 0, -omega(
					0), -omega(1), omega(0), 0);

	cv::Vec3d unit(1., 1., 1.);

	return (cv::Mat().diag(cv::Mat(unit))
			+ (omega_hat / norm_omega) * cv::sin(norm_omega * theta)
			+ ((omega_hat * omega_hat) / (norm_omega * norm_omega))
					* (1 - cv::cos(norm_omega * theta)));

}

double MotionProcessor::triple_product(cv::Vec3d a, cv::Vec3d b, cv::Vec3d c)
{
	return c(0) * (a(1) * b(2) - a(2) * b(1))
			+ c(1) * (a(2) * b(0) - b(2) * a(0))
			+ c(2) * (a(0) * b(1) - b(0) * a(1));
}

cv::Mat MotionProcessor::skew(cv::Vec3d a)
{
	return (cv::Mat_<float>(3, 3) << 0, -a(2), a(1), a(2), 0, -a(0), -a(1), a(
			0), 0);
}

} /* namespace LRM */
