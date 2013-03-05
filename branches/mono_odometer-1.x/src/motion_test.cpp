/*
 * motion_teste.cpp
 *
 *  Created on: Dec 16, 2012
 *      Author: nishitani
 */

#include "core.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <iostream>
#include <cstdio>
#include <list>
#include <vector>
#include <fstream>

using namespace std;
using namespace cv;

void drawMatchesRelative(const vector<KeyPoint>& train,
		const vector<KeyPoint>& query, std::vector<cv::DMatch>& matches,
		Mat& img, const vector<unsigned char>& mask = vector<unsigned char>())
{
	for (int i = 0; i < (int) matches.size(); i++)
	{
		if (mask.empty() || mask[i])
		{
			Point2f pt_new = query[matches[i].queryIdx].pt;
			Point2f pt_old = train[matches[i].trainIdx].pt;
//			Point2f dist = pt_new - pt_old;

			cv::line(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
			cv::circle(img, pt_new, 2, Scalar(255, 0, 125), 1);

		}
	}
}

//Takes a descriptor and turns it into an xy point
void keypoints2points(const vector<KeyPoint>& in, vector<Point2f>& out)
{
	out.clear();
	out.reserve(in.size());
	for (size_t i = 0; i < in.size(); ++i)
	{
		out.push_back(in[i].pt);
	}
}

//Takes an xy point and appends that to a keypoint structure
void points2keypoints(const vector<Point2f>& in, vector<KeyPoint>& out)
{

	out.clear();
	out.reserve(in.size());
	for (size_t i = 0; i < in.size(); ++i)
	{
		out.push_back(KeyPoint(in[i], 1));
	}
}

//Uses computed homography H to warp original input points to new planar position
void warpKeypoints(const Mat& H, const vector<KeyPoint>& in,
		vector<KeyPoint>& out)
{
	vector<Point2f> pts;
	keypoints2points(in, pts);
	vector<Point2f> pts_w(pts.size());
	Mat m_pts_w(pts_w);
	perspectiveTransform(Mat(pts), m_pts_w, H);
	points2keypoints(pts_w, out);
}

//Converts matching indices to xy points
void matches2points(const vector<KeyPoint>& train,
		const vector<KeyPoint>& query, const std::vector<cv::DMatch>& matches,
		std::vector<cv::Point2f>& pts_train, std::vector<Point2f>& pts_query)
{

	pts_train.clear();
	pts_query.clear();
	pts_train.reserve(matches.size());
	pts_query.reserve(matches.size());

	size_t i = 0;

	for (; i < matches.size(); i++)
	{

		const DMatch & dmatch = matches[i];

		pts_query.push_back(query[dmatch.queryIdx].pt);
		pts_train.push_back(train[dmatch.trainIdx].pt);

	}

}

//Normalize the features point
bool feature_point_normalization(std::vector<cv::Point2f> prev_pts,
		std::vector<cv::Point2f> curr_pts,
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
	Tp = cv::Mat(
			cv::Matx33d(sp, 0, -sp * cent_prev.x, 0, sp, -sp * cent_prev.y, 0,
					0, 1));
	Tc = cv::Mat(
			cv::Matx33d(sc, 0, -sc * cent_curr.x, 0, sc, -sc * cent_curr.y, 0,
					0, 1));

	return true;
}

//Estimates the fundamental matrix
cv::Mat compute_F_matrix(std::vector<cv::Point2f> train_pts,
		std::vector<cv::Point2f> query_pts, Mat &mask)
{
	return findFundamentalMat(train_pts, query_pts, FM_RANSAC, 0.1, 0.99, mask);
}

//Extracts rotation and translatipon from essential matrix
vector<cv::Mat> compute_Rt(cv::Mat E, cv::Mat K,
		std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> curr_pts)
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
	SVD svd;
	svd(E);

	Mat tp = svd.u.col(2);
	Mat tn = -svd.u.col(2);
	Mat Ra = svd.u * W * svd.vt;
	Mat Rb = svd.u * W.t() * svd.vt;

	// assure determinant to be positive
	if (cv::determinant(Ra) < 0)
		Ra = -Ra;
	if (cv::determinant(Rb) < 0)
		Rb = -Rb;

	// create vector containing all 4 solutions
	vector<Mat> P; //(4,Mat::eye(4,4,CV_64F));

	Mat P1 = Mat::eye(4, 4, CV_64F);
	Ra.copyTo(P1(cv::Range(0, 3), cv::Range(0, 3)));
	tp.copyTo(P1(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P1);

	Mat P2 = Mat::eye(4, 4, CV_64F);
	Ra.copyTo(P2(cv::Range(0, 3), cv::Range(0, 3)));
	tn.copyTo(P2(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P2);

	Mat P3 = Mat::eye(4, 4, CV_64F);
	Rb.copyTo(P3(cv::Range(0, 3), cv::Range(0, 3)));
	tp.copyTo(P3(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P3);

	Mat P4 = Mat::eye(4, 4, CV_64F);
	Rb.copyTo(P4(cv::Range(0, 3), cv::Range(0, 3)));
	tn.copyTo(P4(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P4);

	return P;
}

//int triangulateCheck(vector<DMatch> matches, Mat &K, Mat P)
//{
//
//	// init 3d point matrix
//	X = Matrix(4, p_matched.size());
//
//	// projection matrices
//	Matrix P1(3, 4);
//	Matrix P2(3, 4);
//	P1.setMat(K, 0, 0);
//	P2.setMat(R, 0, 0);
//	P2.setMat(t, 0, 3);
//	P2 = K * P2;
//
//	// triangulation via orthogonal regression
//	Matrix J(4, 4);
//	Matrix U, S, V;
//	for (int32_t i = 0; i < (int) p_matched.size(); i++)
//	{
//		for (int32_t j = 0; j < 4; j++)
//		{
//			J.val[0][j] = P1.val[2][j] * p_matched[i].u1p - P1.val[0][j];
//			J.val[1][j] = P1.val[2][j] * p_matched[i].v1p - P1.val[1][j];
//			J.val[2][j] = P2.val[2][j] * p_matched[i].u1c - P2.val[0][j];
//			J.val[3][j] = P2.val[2][j] * p_matched[i].v1c - P2.val[1][j];
//		}
//		J.svd(U, S, V);
//		X.setMat(V.getMat(0, 3, 3, 3), 0, i);
//	}
//
//	// compute inliers
//	Matrix AX1 = P1 * X;
//	Matrix BX1 = P2 * X;
//	int32_t num = 0;
//	for (int32_t i = 0; i < X.n; i++)
//		if (AX1.val[2][i] * X.val[3][i] > 0 && BX1.val[2][i] * X.val[3][i] > 0)
//			num++;
//
//	// return number of inliers
//	return num;
//}

void estimate_motion(vector<Point2f> train_pts, vector<Point2f> query_pts, vector<DMatch> matches,
		Mat K, Mat &P)
{
	//Normalization transformations
	Mat Tp, Tc;

	//Normalized vectors
	vector<Point2f> norm_train_pts(train_pts.size());
	vector<Point2f> norm_query_pts(query_pts.size());

	//Fundamental matrix
	Mat F;

	//Inliers mask
	Mat mask;

	if (!feature_point_normalization(train_pts, query_pts, norm_train_pts,
			norm_query_pts, Tp, Tc))
	{
		P = Mat().eye(4, 4, CV_64F);
		return;
	}

	F = compute_F_matrix(norm_train_pts, norm_query_pts, mask);
	if (F.empty())
	{
		P = Mat::eye(4, 4, CV_64F);
		return;
	}

	SVD svd(F);
//	svd.w.at<double>(0) = 1;
//	svd.w.at<double>(1) = 1;
	svd.w.at<double>(2) = 0;

	F = svd.u * cv::Mat().diag(svd.w) * svd.vt;

	Mat E = K.t() * Tc.t() * F * Tp * K;

	cv::Mat R, t;

	vector<Mat> P_vec;
	P_vec = compute_Rt(E, K, norm_train_pts, norm_query_pts);

//	int max_inlier;
//	for (int i = 0; i < P.size(); i++)
//	{
//		int num_inliers = triangulateCheck();
//	}

//
//
//
//	Mat P_curr;
//	int32_t max_inliers = 0;
//	for (int32_t i = 0; i < 4; i++)
//	{
//		int32_t num_inliers = triangulateChieral(p_matched, K, R_vec[i],
//				t_vec[i], X_curr);
//		if (num_inliers > max_inliers)
//		{
//			max_inliers = num_inliers;
//			X = X_curr;
//			R = R_vec[i];
//			t = t_vec[i];
//		}
//	}

}

void resetH(Mat&H)
{
	H = Mat::eye(3, 3, CV_32FC1);
}

/**
 *	main()
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "motion_test");
	std::string node_name = ros::this_node::getName();
	ros::NodeHandle nh(node_name);

	std::string base_path, extension;
	int database_size;

	nh.param<std::string>("base_path", base_path,
			"/home/nishitani/Dropbox/usp/ros/mono_odometer/dataset");
	nh.param<std::string>("extension", extension, ".jpg");
	nh.param<int>("database_size", database_size, 1);

	/****************************/
	/** Feature and Descriptor **/
	/****************************/
	// number of used features
	const int DESIRED_FTRS = 500;

	// Features detector, descriptor and matcher
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> descriptor;
	BFMatcher desc_matcher(NORM_L2); // Matching rule
//	DescriptorMatcher desc_matcher;

	detector = new SiftFeatureDetector(DESIRED_FTRS);
	descriptor = new SiftDescriptorExtractor(128);

	// Feature points
	vector<Point2f> train_pts, query_pts;

	// Keypoints
	vector<KeyPoint> train_kpts, query_kpts;

	// Descriptors
	Mat train_desc, query_desc;

	// Matches
	vector<DMatch> matches;
	vector<vector<DMatch> > radiusMatches;

	// Mask of near matches
	vector<unsigned char> match_mask;
	/****************************/

	/****************************/
	/** Images ******************/
	/****************************/
	Mat frame, output_image, current_image, previous_image;

	string window_name("Matches");
	namedWindow(window_name.c_str(), CV_WINDOW_NORMAL);
	/****************************/

	Mat H_prev = Mat::eye(3, 3, CV_32FC1);

	/************************/
	/** Calibration Matrix **/
	/************************/
	Mat K;
	FileStorage calib_data;
	calib_data.open(
			"/home/nishitani/Dropbox/usp/ros/mono_odometer/calib/camera_left.yaml",
			cv::FileStorage::READ);
	calib_data["cameraMatrix"] >> K;
	/************************/

	for (int i = 1; i < database_size + 1; i++)
	{
		/************************/
		/**Leitura da Image *****/
		/************************/
		char img_path[100];
		sprintf(img_path, "%s%04d%s", base_path.c_str(), i, extension.c_str());
		cout << img_path << endl;

		current_image = imread(string(img_path), CV_LOAD_IMAGE_GRAYSCALE);
		/************************/

		detector->detect(current_image, query_kpts);
		descriptor->compute(current_image, query_kpts, query_desc);

		//Projection matrix
		Mat P;
		//Rotation matrix
//		Matx33f R;
		//traslation vector
//		Vec3f t;

		if (!train_desc.empty())
		{
			/*********************************/
			/** Calcula os matches proximos **/
			/*********************************/
			desc_matcher.radiusMatch(train_desc, query_desc, radiusMatches,
					100.0);
			matches.clear();
			for (int i = 0; i < (int) radiusMatches.size(); i++)
			{
				if (radiusMatches[i].size())
				{
					float dist = 10000.0;
					int pos = 0;
					for (int j = 0; j < (int) radiusMatches[i].size(); j++)
					{
						if (dist > radiusMatches[i][j].distance)
						{
							dist = radiusMatches[i][j].distance;
							pos = j;
						}
					}
					matches.push_back(radiusMatches[i][pos]);
				}
			}
			/*********************************/

			drawMatches(previous_image, train_kpts, current_image, query_kpts,
					matches, output_image, Scalar::all(-1), Scalar::all(-1),
					vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

			matches2points(train_kpts, query_kpts, matches, train_pts,
					query_pts);

			estimate_motion(train_pts, query_pts, matches, K, P);

//			R = P(Range::all(),Range(0,3));
//			t = P.col(3);
//
//			float ry = asin( -R(2,0) ),
//				  rx = asin( R(2,1)/cos(ry) ),
//				  rz = asin( R(1,0)/cos(ry) );
//			float cx = cos(rx),cy = cos(ry),cz = cos(rz),
//				  sx = sin(rx),sy = sin(ry),sz = sin(rz);
//
//			Mat C(4,4,CV_32F);
//			C.at<float>(0,0) = +cy*cz;          C.at<float>(0,1) = -cy*sz;          C.at<float>(0,2) = +sy;    C.at<float>(0,3) = t(0);
//			C.at<float>(1,0) = +sx*sy*cz+cx*sz; C.at<float>(1,1) = -sx*sy*sz+cx*cz; C.at<float>(1,2) = -sx*cy; C.at<float>(1,3) = t(1);
//			C.at<float>(2,0) = -cx*sy*cz+sx*sz; C.at<float>(2,1) = +cx*sy*sz+sx*cz; C.at<float>(2,2) = +cx*cy; C.at<float>(2,3) = t(2);
//			C.at<float>(3,0) = 0;               C.at<float>(3,1) = 0;               C.at<float>(3,2) = 0;      C.at<float>(3,3) = 1;
//
//			C = C.inv();
//
//			cout << C << endl;
//
//			R = C( Range(0,3),Range(0,3) ); R = R.t();
//			t = C( Range(0,3),Range(3,4) );
//
//			pose = R*pose+t;
		}

		train_kpts = query_kpts;
		query_desc.copyTo(train_desc);
		current_image.copyTo(previous_image);

		if (!output_image.empty())
			imshow(window_name.c_str(), output_image);

		if ((char) waitKey(0) == 'q')
			break;
	}
}
