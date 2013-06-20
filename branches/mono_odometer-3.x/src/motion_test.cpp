/*
 * motion_teste.cpp
 *
 *  Created on: Dec 16, 2012
 *      Author: nishitani
 */

#include "core.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cstdio>
#include <list>
#include <vector>
#include <fstream>

using namespace std;
using namespace cv;

double rad2deg(double rad)
{
	return (rad * 180 / CV_PI);
}

void compare(vector<Point2d> train_pts, vector<Point2d> query_pts,
		vector<DMatch> matches, const vector<KeyPoint>& train_kpts,
		const vector<KeyPoint>& query_kpts)
{
	int sizes[3] =
	{ train_pts.size(), query_pts.size(), matches.size() };
	int vec_size = sizes[0];
	for (int i = 0; i < 3; i++)
	{
		if (vec_size != sizes[i])
		{
			cout << "Train PTS  " << sizes[0] << endl << "Query PTS  "
					<< sizes[1] << endl << "Matches    " << sizes[2] << endl;
			printf("Error: Wrong size\n");
			return;
		}
	}
	for (int i = 0; i < (int) matches.size(); i++)
	{
		const DMatch & dmatch = matches[i];
		if (train_pts[i].x != train_kpts[dmatch.trainIdx].pt.x
				|| train_pts[i].y != train_kpts[dmatch.trainIdx].pt.y
				|| query_pts[i].x != query_kpts[dmatch.queryIdx].pt.x
				|| query_pts[i].y != query_kpts[dmatch.queryIdx].pt.y)
		{
			cout << "PTS : " << train_pts[i] << ' ' << query_pts[i] << endl;
			cout << "KPTS: " << train_kpts[dmatch.trainIdx].pt << ' '
					<< query_kpts[dmatch.queryIdx].pt << endl;
			cout << endl;
		}
	}
}

void drawMatchesRelative(const vector<KeyPoint>& train,
		const vector<KeyPoint>& query, std::vector<cv::DMatch>& matches,
		Mat& img, const vector<char>& mask = vector<char>())
{
	for (int i = 0; i < (int) matches.size(); i++)
	{
		if (!mask.empty())
		{
			if (!mask[i])
			{
				Point2d pt_new = query[matches[i].queryIdx].pt;
				Point2d pt_old = train[matches[i].trainIdx].pt;

				cv::line(img, pt_new, pt_old, Scalar(125, 125, 255), 1);
				cv::circle(img, pt_new, 2, Scalar(0, 0, 255), 1);
			}
			else
			{
				Point2d pt_new = query[matches[i].queryIdx].pt;
				Point2d pt_old = train[matches[i].trainIdx].pt;

				cv::line(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
				cv::circle(img, pt_new, 2, Scalar(0, 255, 0), 1);
			}
		}
		else
		{
			Point2d pt_new = query[matches[i].queryIdx].pt;
			Point2d pt_old = train[matches[i].trainIdx].pt;

			cv::line(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
			cv::circle(img, pt_new, 2, Scalar(0, 255, 0), 1);
		}
	}
}

void drawMatchesRelative(const vector<Point2d> train,
		const vector<Point2d> query, Mat input, Mat& output,
		const vector<char>& mask = vector<char>())
{
	int matches_size = train.size();
	cvtColor(input, output, CV_GRAY2BGR, 3);

	if (!mask.empty())
	{
		for (int i = 0; i < matches_size; i++)
		{
			if (!mask[i])
			{
				line(output, query[i], train[i], Scalar(0, 0, 255), 1, CV_AA);
				circle(output, query[i], 2, Scalar(125, 125, 255), 1, CV_AA);
			}
		}
		for (int i = 0; i < matches_size; i++)
		{
			if (mask[i])
			{
				line(output, query[i], train[i], Scalar(0, 255, 0), 1, CV_AA);
				circle(output, query[i], 2, Scalar(125, 255, 125), 1, CV_AA);
			}
		}
	}
	else
	{
		for (int i = 0; i < matches_size; i++)
		{
			line(output, query[i], train[i], Scalar(0, 255, 0), 1, CV_AA);
			circle(output, query[i], 2, Scalar(125, 255, 125), 1, CV_AA);
		}
	}
}

//Takes a descriptor and turns it into an xy point
void keypoints2points(const vector<KeyPoint>& in, vector<Point2d>& out)
{
	out.clear();
	out.reserve(in.size());
	for (size_t i = 0; i < in.size(); ++i)
	{
		out.push_back(in[i].pt);
	}
}

//Takes an xy point and appends that to a keypoint structure
void points2keypoints(const vector<Point2d>& in, vector<KeyPoint>& out)
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
	vector<Point2d> pts;
	keypoints2points(in, pts);
	vector<Point2d> pts_w(pts.size());
	Mat m_pts_w(pts_w);
	perspectiveTransform(Mat(pts), m_pts_w, H);
	points2keypoints(pts_w, out);
}

//Converts matching indices to xy points
void matches2points(const vector<KeyPoint>& train,
		const vector<KeyPoint>& query, const std::vector<cv::DMatch>& matches,
		vector<cv::Point2d>& pts_train, vector<Point2d>& pts_query)
{

	pts_train.clear();
	pts_query.clear();
	pts_train.reserve(matches.size() + 100);
	pts_query.reserve(matches.size() + 100);

	for (int i = 0; i < (int) matches.size(); i++)
	{

		const DMatch & dmatch = matches[i];

		pts_query.push_back(query[dmatch.queryIdx].pt);
		pts_train.push_back(train[dmatch.trainIdx].pt);
	}
}

//Normalize the features point
//Chojnacki, W., Brooks, M. J., Van den Hengel, a., & Gawley, D. (2003). Revisiting hartley’s normalized eight-point algorithm.
bool feature_point_normalization(std::vector<cv::Point2d> prev_pts,
		std::vector<cv::Point2d> curr_pts,
		std::vector<cv::Point2d> &norm_prev_pts,
		std::vector<cv::Point2d> &norm_curr_pts, cv::Mat &Tp, cv::Mat &Tc)
{
	uint matches_size = prev_pts.size();
	cv::Point2d cent_prev(0, 0), cent_curr(0, 0);
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
	Tp = Mat(
			Matx33d(sp, 0, -sp * cent_prev.x, 0, sp, -sp * cent_prev.y, 0, 0,
					1));
	Tc = Mat(
			Matx33d(sc, 0, -sc * cent_curr.x, 0, sc, -sc * cent_curr.y, 0, 0,
					1));

	return true;
}
//Estimates the fundamental matrix

cv::Mat compute_F_matrix(std::vector<cv::Point2d> train_pts,
		std::vector<cv::Point2d> query_pts, Mat &mask)
{
	return findFundamentalMat(train_pts, query_pts, FM_RANSAC, 0.01, 0.99, mask);
}

//cv::Mat compute_F_matrix(std::vector<cv::Point2d> train_pts,
//		std::vector<cv::Point2d> query_pts, Mat &mask)
//{
//	int N = train_pts.size();
//	// create constraint matrix A
//	Mat A(N,9,CV_64F);
//	for (uint i=0; i<N; i++) {
//	  A.at<double>(i,0) = query_pts[i].x*train_pts[i].x;
//	  A.at<double>(i,1) = query_pts[i].x*train_pts[i].y;
//	  A.at<double>(i,2) = query_pts[i].x;
//	  A.at<double>(i,3) = query_pts[i].y*train_pts[i].x;
//	  A.at<double>(i,4) = query_pts[i].y*train_pts[i].y;
//	  A.at<double>(i,5) = query_pts[i].y;
//	  A.at<double>(i,6) = train_pts[i].x;
//	  A.at<double>(i,7) = train_pts[i].y;
//	  A.at<double>(i,8) = 1;
//	}
//
//	// compute singular value decomposition of A
//	SVD svd;
//	svd(A);
//
//	// extract fundamental matrix from the column of V corresponding to the smallest singular value
//	Mat F = svd.vt.row(8).reshape(1,3);
//	F = F.reshape(1,3);
//	cout << F << endl;
//
//	return F;
//}

//Extracts rotation and translatipon from essential matrix
vector<cv::Mat> compute_Rt(cv::Mat E, cv::Mat K)
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

	Mat P1 = Mat::eye(3, 4, CV_64F);
	Ra.copyTo(P1(cv::Range(0, 3), cv::Range(0, 3)));
	tp.copyTo(P1(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P1);

	Mat P2 = Mat::eye(3, 4, CV_64F);
	Ra.copyTo(P2(cv::Range(0, 3), cv::Range(0, 3)));
	tn.copyTo(P2(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P2);

	Mat P3 = Mat::eye(3, 4, CV_64F);
	Rb.copyTo(P3(cv::Range(0, 3), cv::Range(0, 3)));
	tp.copyTo(P3(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P3);

	Mat P4 = Mat::eye(3, 4, CV_64F);
	Rb.copyTo(P4(cv::Range(0, 3), cv::Range(0, 3)));
	tn.copyTo(P4(cv::Range(0, 3), cv::Range(3, 4)));
	P.push_back(P4);

	return P;
}

int triangulateCheck(vector<Point2d> train_pts, vector<Point2d> query_pts,
		Mat &K, Mat P, vector<char> mask = vector<char>())
{
	// init 3d point matrix
	Mat A = Mat::zeros(4, 4, CV_64F);
	Mat P1, P2;
	Mat X_vec;
	SVD svd;

	int num_inliers = 0;

	// projection matrices
	P1 = K * Mat::eye(3, 4, CV_64F);
	P2 = K * P;

	// triangulation via orthogonal regression
	for (int i = 0; i < (int) mask.size(); i++)
	{
//		int i = 1;
		if (!(int) mask[i])
		{
			continue;
		}
		A.row(0) += (query_pts[i].x * P1.row(2).t() - P1.row(0).t()).t();
		A.row(1) += (query_pts[i].y * P1.row(2).t() - P1.row(1).t()).t();
		A.row(2) += (train_pts[i].x * P2.row(2).t() - P2.row(0).t()).t();
		A.row(3) += (train_pts[i].y * P2.row(2).t() - P2.row(1).t()).t();

		svd(A);

		Mat X(svd.vt.row(3));

		/**************************/
		/** Checa a profundidade **/
		/**************************/
		Mat x1 = P1 * X.t(), x2 = P2 * X.t();
//		Mat M1(P1, Rect(0, 0, 3, 3)), M2(P2, Rect(0, 0, 3, 3));
//		int sign1 = determinant(M1) > 0 ? 1 : -1, sign2 =
//				determinant(M2) > 0 ? 1 : -1;
//
//		double depth1, depth2;
//		depth1 = (sign1 * x1.at<double>(2, 0))
//				/ (X.at<double>(0, 3) * norm(M1.col(2)));
//		depth2 = (sign2 * x2.at<double>(2, 0))
//				/ (X.at<double>(0, 3) * norm(M2.col(2)));
//		if (depth1 > 0 && depth2 > 0)
//			num_inliers++;
		if (x1.at<double>(2, 0) * X.at<double>(0, 3) > 0
				&& x2.at<double>(2, 0) * X.at<double>(0, 3) > 0)
			num_inliers++;
		/**************************/

		X /= X.at<double>(0, 3);

		X_vec.push_back(X);

		A = Mat::zeros(4, 4, CV_64F);
	}

	// return number of inliers
	return num_inliers;
}

void estimate_motion(vector<Point2d> train_pts, vector<Point2d> query_pts,
		vector<DMatch> matches, Mat K, Mat &P, vector<char> &inlier_mask)
{
	//Normalization transformations
	Mat Tp, Tc;

	//Normalized vectors
	vector<Point2d> norm_train_pts(train_pts.size());
	vector<Point2d> norm_query_pts(query_pts.size());

	//Fundamental matrix
	Mat F;

	//Inliers mask
	Mat mask;

//	for(uint i=0 ; i<train_pts.size() ; i++){
//		train_pts[i] = Point2d(Mat(K.inv()*Mat(train_pts[i])));
//		query_pts[i] = Point2d(Mat(K.inv()*Mat(query_pts[i])));
//	}

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
	svd.w.at<double>(0) = 1;
	svd.w.at<double>(1) = 1;
	svd.w.at<double>(2) = 0;

	F = svd.u * cv::Mat().diag(svd.w) * svd.vt;

	Mat E = K.t() * Tc.t() * F * Tp * K;

//	cout << E << endl;

	cv::Mat R, t;

	vector<Mat> P_vec;
	P_vec = compute_Rt(E, K);

	int max_inlier = 0;
	int pos = 0;

	vector<vector<char> > mask_vec;

	for (size_t i = 0; i < P_vec.size(); i++)
	{
		vector<char> _mask(mask);
		int inlier = triangulateCheck(train_pts, query_pts, K, P_vec[i], _mask);
		printf("%d: %d\n", i, inlier);
		cout << P_vec[i] << endl;
		if (max_inlier < inlier)
		{
			max_inlier = inlier;
			pos = i;
		}
		mask_vec.push_back(_mask);
	}
	P = P_vec[pos](Range::all(), Range::all());

	inlier_mask = mask_vec[pos];
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

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(
			"/mono_odometer/odom", 1);
	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformListener tf_listener;

	image_transport::Publisher outlier_pub; ///< Image publisher
	image_transport::Publisher match_pub; ///< Image publisher
	image_transport::Publisher original_pub; ///< Image publisher

	image_transport::ImageTransport it(nh);

	outlier_pub = it.advertise("/image/outlier", 1);
	match_pub = it.advertise("/image/match", 1);
	original_pub = it.advertise("/image/original", 1);

	std::string base_path, extension, calib_path;
	int database_size, feature_type;

	nh.param<std::string>("base_path", base_path,
			"/home/nishitani/Dropbox/usp/ros/mono_odometer/dataset");
	nh.param<std::string>("calib_path", calib_path,
			"~/ros/mono_odometer/calib/camera.yaml");
	nh.param<std::string>("extension", extension, ".jpg");
	nh.param<int>("database_size", database_size, 1);
	nh.param<int>("feature_type", feature_type, 0);

	/****************************/
	/** Feature and Descriptor **/
	/****************************/
	// number of used features
	const int DESIRED_FTRS = 500;
	// Features detector, descriptor and matcher
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> descriptor;
	BFMatcher desc_matcher(NORM_L2, true); // Matching rule
//	DescriptorMatcher desc_matcher;

	switch (feature_type)
	{
	case 0:
		//SIFT
		detector = new SiftFeatureDetector();
		descriptor = new SiftDescriptorExtractor();
		break;
	case 1:
		//FAST
//		detector = new cv::GridAdaptedFeatureDetector( new cv::FastFeatureDetector(10, true), DESIRED_FTRS, 4, 4);
		detector = new cv::PyramidAdaptedFeatureDetector(
				new cv::FastFeatureDetector(5, true));
		descriptor = new cv::BriefDescriptorExtractor();
		break;
	case 2:
		//SHI
		detector = new GoodFeaturesToTrackDetector(DESIRED_FTRS, 0.01, 1.0, 5);
		descriptor = new BriefDescriptorExtractor();
		break;
	}

	// Feature points
	vector<Point2d> train_pts, query_pts;

	// Keypoints
	vector<KeyPoint> train_kpts, query_kpts, matched_train_kpts,
			matched_query_kpts;

	// Descriptors
	Mat train_desc, query_desc;

	// Matches
	vector<DMatch> matches;
	vector<vector<DMatch> > radiusMatches;

	// Mask of near matches
	vector<unsigned char> match_mask;
	vector<char> inlier_mask;
	/****************************/

	/****************************/
	/** Images ******************/
	/****************************/
	Mat frame, pair_image, relative_image, current_image, previous_image;

	string pair_window_name("Matches");
	string relative_window_name("Relative");
//	namedWindow(pair_window_name.c_str(), CV_WINDOW_NORMAL);
	namedWindow(relative_window_name.c_str(), CV_WINDOW_NORMAL);
	/****************************/

	/************************/
	/** Calibration Matrix **/
	/************************/
	Mat K;
	FileStorage calib_data;
	calib_data.open(calib_path.c_str(), cv::FileStorage::READ);
	calib_data["cameraMatrix"] >> K;
	/************************/

	/************************/
	/** Motion Structures  **/
	/************************/
	//Projection matrix
	Mat P = Mat::eye(4, 4, CV_64F);

	//Rotation matrix
	Matx33d R;
	//traslation vector
	Vec3d t, _pose(0, 0, 0);
	//Pose vector
	tf::Pose pose;
	pose.setIdentity();
	odom_broadcaster.sendTransform(
			tf::StampedTransform(pose, ros::Time::now(), "odom", "base_link"));
	/************************/

	/*****************************/
	//TESTING
//	Mat x1, x2;
//	FileStorage cube_pts;
//	cube_pts.open(
//			"/home/nishitani/ros/mono_odometer/dataset/other/raw/01/cube_pts.yaml",
//			cv::FileStorage::READ);
//	cube_pts["query"] >> x1;
//	cube_pts["train"] >> x2;
////	x1 = x1.t();
////	x2 = x2.t();
////	cout << x1 << endl << endl << x2 << endl;
//
//	vector<Point2d> X1, X2;
//
//	for (int i = 0; i < x1.cols; i++)
//	{
////		X1.push_back(Point2d((double)x1.col(i).data[0],(double)x1.col(i).data[1]));
////		X2.push_back(Point2d((double)x2.col(i).data[0],(double)x2.col(i).data[1]));
//		X1.push_back(Point2d(x1.col(i)));
//		X2.push_back(Point2d(x2.col(i)));
////		cout << x1.col(i) << endl << x2.col(i) << endl << endl;
//	}
	/*****************************/

	ROS_INFO("BEGIN");

	for (int i = 1;; i++)
	{
		ROS_INFO("RUN %d",i);

		/************************/
		/**Leitura da Image *****/
		/************************/
		char img_path[100];
		sprintf(img_path, "%s%04d%s", base_path.c_str(), i, extension.c_str());

		current_image = imread(string(img_path), CV_LOAD_IMAGE_GRAYSCALE);
		if (current_image.empty())
			break;
		/************************/

		detector->detect(current_image, query_kpts);
		descriptor->compute(current_image, query_kpts, query_desc);

		if (!train_desc.empty())
		{

//			desc_matcher.match(query_desc, train_desc,matches);
			/******************************************/
			/** Calcula as correspondencias proximas **/
			/******************************************/
			desc_matcher.radiusMatch(query_desc, train_desc, radiusMatches,
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
			/******************************************/

			drawMatches(current_image, query_kpts, previous_image, train_kpts,
					matches, pair_image, Scalar::all(-1), Scalar::all(-1),
					vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			if (matches.size() > 8)
			{

				matches2points(train_kpts, query_kpts, matches, train_pts,
						query_pts);

				estimate_motion(train_pts, query_pts, matches, K, P,
						inlier_mask);

				drawMatchesRelative(train_pts, query_pts, current_image,
						relative_image, inlier_mask);

				R = P(Range::all(), Range(0, 3));
				t = P.col(3);

				double ry = asin(R(0, 2)), rx = asin(-R(1, 2) / cos(ry)), rz =
						asin(-R(0, 1) / cos(ry));
				double cx = cos(rx), cy = cos(ry), cz = cos(rz), sx = sin(rx),
						sy = sin(ry), sz = sin(rz);

				Mat C(4, 4, CV_64F);
				C.at<double>(0, 0) = +cy * cz;
				C.at<double>(0, 1) = -cy * sz;
				C.at<double>(0, 2) = +sy;
				C.at<double>(0, 3) = t(0);
				C.at<double>(1, 0) = +sx * sy * cz + cx * sz;
				C.at<double>(1, 1) = -sx * sy * sz + cx * cz;
				C.at<double>(1, 2) = -sx * cy;
				C.at<double>(1, 3) = t(1);
				C.at<double>(2, 0) = -cx * sy * cz + sx * sz;
				C.at<double>(2, 1) = +cx * sy * sz + sx * cz;
				C.at<double>(2, 2) = +cx * cy;
				C.at<double>(2, 3) = t(2);
				C.at<double>(3, 0) = 0;
				C.at<double>(3, 1) = 0;
				C.at<double>(3, 2) = 0;
				C.at<double>(3, 3) = 1;
				C = C.inv();

				R = C(Range(0, 3), Range(0, 3)); //R = R.t();
				t = C(Range(0, 3), Range(3, 4));

				printf("rx=%f - ry=%f - rz=%f\n\n", rad2deg(rx), rad2deg(ry),
						rad2deg(rz));

				_pose = R * _pose + t;

				tf::Matrix3x3 _R(R.val[0], R.val[1], R.val[2], R.val[3],
						R.val[4], R.val[5], R.val[6], R.val[7], R.val[8]);
				tf::Vector3 _T(t(0), t(1), t(2));
				tf::Transform motion(_R, _T);

				std::string error_msg;
				tf::StampedTransform base_to_sensor;
				if (tf_listener.canTransform("base_link", "camera",
						ros::Time(0), &error_msg))
				{
					tf_listener.lookupTransform("base_link", "camera",
							ros::Time(0), base_to_sensor);
				}
				else
				{
					ROS_WARN_THROTTLE(10.0,
							"The tf from '%s' to '%s' does not seem to be available, "
							"will assume it as identity!", "base_link", "camera");
					ROS_DEBUG("Transform error: %s", error_msg.c_str());
					base_to_sensor.setIdentity();
				}

				motion = base_to_sensor * motion * base_to_sensor.inverse();

				pose *= motion;

				odom_broadcaster.sendTransform(
						tf::StampedTransform(pose, ros::Time::now(), "odom",
								"base_link"));

				//next, we'll publish the odometry message over ROS
				nav_msgs::Odometry odom;
				odom.header.stamp = ros::Time::now();
				odom.header.frame_id = "odom";
				odom.child_frame_id = "camera";

				tf::poseTFToMsg(pose, odom.pose.pose);

				printf("odom: %f %f %f\n", odom.pose.pose.position.x,
						odom.pose.pose.position.y, odom.pose.pose.position.z);

				double delta_t = 1.0;
				odom.twist.twist.linear.x = motion.getOrigin().getX() / delta_t;
				odom.twist.twist.linear.y = motion.getOrigin().getY() / delta_t;
				odom.twist.twist.linear.z = motion.getOrigin().getZ() / delta_t;
				double yaw, pitch, roll;
				motion.getBasis().getEulerYPR(yaw, pitch, roll);
				odom.twist.twist.angular.x = roll / delta_t;
				odom.twist.twist.angular.y = pitch / delta_t;
				odom.twist.twist.angular.z = yaw / delta_t;
				//publish the message
				odom_pub.publish(odom);

			}
		}

		train_kpts = query_kpts;
		query_desc.copyTo(train_desc);
		current_image.copyTo(previous_image);

		if (!pair_image.empty())
		{
//			imshow(pair_window_name.c_str(), pair_image);
			cv_bridge::CvImage img;
			img.image = pair_image;
			img.header.stamp = ros::Time::now();
			img.header.seq = i;
			img.header.frame_id = std::string("image_" + i);
			img.encoding = sensor_msgs::image_encodings::BGR8;
			match_pub.publish(img.toImageMsg());
		}
		if (!relative_image.empty())
		{
//			imshow(relative_window_name.c_str(), relative_image);
			cv_bridge::CvImage img;
			img.image = relative_image;
			img.header.stamp = ros::Time::now();
			img.header.seq = i;
			img.header.frame_id = std::string("image_" + i);
			img.encoding = sensor_msgs::image_encodings::BGR8;
			outlier_pub.publish(img.toImageMsg());
		}

		if ((char) waitKey(0) == 'q')
			break;
	}
	ros::shutdown();
}
