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

	nh.param<double>("RANSAC_EPIPOLAR_DIST", value_double, 0.00001);
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
	epipolar_dist = 0.00001;
}

MotionProcessor::~MotionProcessor()
{
}

int MotionProcessor::setting(MotionProcessorParameter param)
{
	int MP_ERR_CODE = 0; //Error Code

	epipolar_dist = param.getParameterByName<double>("RANSAC_EPIPOLAR_DIST");
	confidence = param.getParameterByName<double>("RANSAC_CONFIDENCE");

	srand(time(NULL));

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
	// Order the matches if needed
	int m_size = matches.size();
	if (!matches.empty())
	{
		std::vector<cv::Point2d> t, q;
		for (int i = 0; i < m_size; i++)
		{
			cv::DMatch m = matches[i];
			t.push_back(train_pts[m.trainIdx]);
			q.push_back(query_pts[m.queryIdx]);
		}
		train_pts = t;
		query_pts = q;
	}

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

	std::vector<cv::Point2d> _train_pts, _query_pts;
	inliers.clear();
	for (int i = 0; i < 1000; i++)
	{
		getRandSample(8, norm_train_pts, norm_query_pts, _train_pts,
				_query_pts);

		F = compute_F_matrix(_train_pts, _query_pts);

		std::vector<char> _inliers = score(F, norm_train_pts, norm_query_pts);
		if (inliers.empty()
				|| cv::countNonZero(_inliers) > cv::countNonZero(inliers))
		{
			inliers = _inliers;
		}
	}

	if (cv::countNonZero(inliers) > 8)
	{
		_train_pts.clear();
		_query_pts.clear();
		for (uint i = 0; i < norm_train_pts.size(); i++)
		{
			if (inliers[i])
			{
				_train_pts.push_back(norm_train_pts[i]);
				_query_pts.push_back(norm_query_pts[i]);
			}
		}
		cv::Mat tp = cv::Mat(_train_pts).reshape(1, _train_pts.size()), qp =
				cv::Mat(_query_pts).reshape(1, _query_pts.size());
		F = compute_F_matrix(_train_pts, _query_pts);
	}
	else
	{
		P = cv::Mat::eye(4, 4, CV_64F);
		return;
	}

	if (cv::countNonZero(F) < 2)
	{
		P = cv::Mat::eye(4, 4, CV_64F);
		return;
	}

	cv::Mat E = K.t() * Tc.t() * F * Tp * K;

	cv::Mat R, t;

	std::vector<cv::Mat> P_vec;
	P_vec = compute_Rt(E, K);

	int max_inliers = 0;
	int pos = -1;

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
		double ry = asin(P_vec[i].at<double>(0, 2));
		double rx = asin(-P_vec[i].at<double>(1, 2) / cos(ry));
		double rz = asin(-P_vec[i].at<double>(0, 1) / cos(ry));;
		ROS_INFO("Angles of the option %d: (%f,%f,%f)",i,rx,ry,rz);
	}
	printf("\n");

	if (pos < 0)
	{
		P = cv::Mat::eye(4, 4, CV_64F);
		return;
	}

	P = P_vec[pos];
	ROS_INFO_STREAM(
			"R:\n" << P(cv::Range(0,3),cv::Range(0,3)) << "\nt:\n" << P(cv::Range(0,3),cv::Range(3,4)) << std::endl);

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
// number of active p_matched
	int N = train_pts.size();

// create constraint matrix A
	cv::Mat A(N, 9, CV_64F);

	for (int i = 0; i < N; i++)
	{
//    Matcher::p_match m = p_matched[active[i]];
		A.at<double>(i, 0) = query_pts[i].x * train_pts[i].x;
		A.at<double>(i, 1) = query_pts[i].x * train_pts[i].y;
		A.at<double>(i, 2) = query_pts[i].x;
		A.at<double>(i, 3) = query_pts[i].y * train_pts[i].x;
		A.at<double>(i, 4) = query_pts[i].y * train_pts[i].y;
		A.at<double>(i, 5) = query_pts[i].y;
		A.at<double>(i, 6) = train_pts[i].x;
		A.at<double>(i, 7) = train_pts[i].y;
		A.at<double>(i, 8) = 1;
	}

// compute singular value decomposition of A
//  Mat U,W,V;
	cv::SVD svd(A, cv::SVD::FULL_UV);

// extract fundamental matrix from the column of V corresponding to the smallest singular value
	cv::Mat F = cv::Mat(svd.vt.row(8)); // Matrix::reshape(V.getMat(0,8,8,8),3,3);
	F = F.reshape(1, 3);

	svd(F);

	/**
	 * @todo Find out why the hell W=(1,1,0) fails
	 */
	svd.w.at<double>(2) = 0;

	F = svd.u * cv::Mat::diag(svd.w) * svd.vt;

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
	cv::Mat W = cv::Mat(3, 3, CV_64F, w);

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
		{
			continue;
		}

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
		{
			num_inliers++;
		}
		else
		{
			mask[i] = 0;
		}
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
	P = cv::Mat::eye(4, 4, CV_64F);
	return 0;
}

int MotionProcessor::getRandSample(int size,
		std::vector<cv::Point2d> norm_train_pts,
		std::vector<cv::Point2d> norm_query_pts,
		std::vector<cv::Point2d> &_train_pts,
		std::vector<cv::Point2d> &_query_pts)
{
	std::vector<int> used(norm_train_pts.size(), 0);

	_train_pts.clear();
	_query_pts.clear();

	for (int i = 0; i < size; i++)
	{
		int p = rand() % norm_train_pts.size();
		while (used[p])
			p = rand() % norm_train_pts.size();
		_train_pts.push_back(norm_train_pts[p]);
		_query_pts.push_back(norm_query_pts[p]);
		used[p] = 1;
	}

	return size;
}

std::vector<char> MotionProcessor::score(cv::Mat F,
		std::vector<cv::Point2d> train, std::vector<cv::Point2d> query)
{
//Based on the libviso2 getInlier() function

//	std::cout << "F:\n" << F << std::endl;

	int n = train.size();
	std::vector<double> f = cv::Mat_<double>(F.reshape(1, 1));

// loop variables
	double x2tFx1;
	double Fx1u, Fx1v, Fx1w;
	double Ftx2u, Ftx2v;

// vector with inliers
	std::vector<char> inliers(n, 0);

// for all matches do
	for (int i = 0; i < n; i++)
	{
//		printf("Train (%f,%f) - Query (%f,%f)\n", train[i].x,
//				train[i].y, query[i].x, query[i].y);

		// F*x1
		Fx1u = f[0] * train[i].x + f[1] * train[i].y + f[2];
		Fx1v = f[3] * train[i].x + f[4] * train[i].y + f[5];
		Fx1w = f[6] * train[i].x + f[7] * train[i].y + f[8];

//		printf("Fx1u = %f\nFx1v = %f\nFx1w = %f\n", Fx1u,Fx1v,Fx1w);

		// F'*x2
		Ftx2u = f[0] * query[i].x + f[3] * query[i].y + f[6];
		Ftx2v = f[1] * query[i].x + f[4] * query[i].y + f[7];

//		printf("Ftx2u = %f\nFtx2v = %f\n", Ftx2u,Ftx2v);

		// x2'*F*x1
		x2tFx1 = query[i].x * Fx1u + query[i].y * Fx1v + Fx1w;

//		printf("x2tFx1 = %f\n", x2tFx1);

		// sampson distance
		double d = x2tFx1 * x2tFx1
				/ (Fx1u * Fx1u + Fx1v * Fx1v + Ftx2u * Ftx2u + Ftx2v * Ftx2v);

//		printf("Error = %f\n", d);
//				getchar();

		// check threshold
		if (fabs(d) < epipolar_dist)
			inliers[i] = 1;
	}

// return set of all inliers
	return inliers;
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
