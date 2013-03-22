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
			if (mask[i])
			{
				Point2d pt_new = query[matches[i].queryIdx].pt;
				Point2d pt_old = train[matches[i].trainIdx].pt;

				cv::line(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
				cv::circle(img, pt_new, 2, Scalar(0, 255, 0), 1);
			}
			else
			{
				Point2d pt_new = query[matches[i].queryIdx].pt;
				Point2d pt_old = train[matches[i].trainIdx].pt;

				cv::line(img, pt_new, pt_old, Scalar(125, 125, 255), 1);
				cv::circle(img, pt_new, 2, Scalar(0, 0, 255), 1);
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
			if (mask[i])
			{
				line(output, query[i], train[i], Scalar(0, 255, 0), 1, CV_AA);
				circle(output, query[i], 2, Scalar(125, 255, 125), 1, CV_AA);
			}
			else
			{
				line(output, query[i], train[i], Scalar(0, 0, 255), 1, CV_AA);
				circle(output, query[i], 2, Scalar(125, 125, 255), 1, CV_AA);
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
	P1 = Mat::eye(3, 4, CV_64F);
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
		Mat M1(P1, Rect(0, 0, 3, 3)), M2(P2, Rect(0, 0, 3, 3));
		int sign1 = determinant(M1) > 0 ? 1 : -1, sign2 =
				determinant(M2) > 0 ? 1 : -1;

		double depth1 = (sign1 * x1.at<double>(2, 0))
				/ (X.at<double>(0, 3) * norm(M1.col(2))), depth2 = (sign2
				* x2.at<double>(2, 0)) / (X.at<double>(0, 3) * norm(M2.col(2)));
		if (depth1 > 0 && depth2 > 0)
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
//	const int DESIRED_FTRS = 500;

	// Features detector, descriptor and matcher
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> descriptor;
	BFMatcher desc_matcher(NORM_L2,true); // Matching rule
//	DescriptorMatcher desc_matcher;

	//SIFT
//	detector = new SiftFeatureDetector();
//	descriptor = new SiftDescriptorExtractor();

	//SURF
//	detector = new SurfFeatureDetector(1000);
//	descriptor = new SurfDescriptorExtractor();

	//ORB
//	detector = new cv::ORB( DESIRED_FTRS );
//	descriptor = new cv::OrbDescriptorExtractor( DESIRED_FTRS );

	//FAST
//	detector = new cv::GridAdaptedFeatureDetector( new cv::FastFeatureDetector(10, true), DESIRED_FTRS, 4, 4);
	detector = new cv::PyramidAdaptedFeatureDetector( new cv::FastFeatureDetector(5, true) );
	descriptor = new cv::BriefDescriptorExtractor();

	//SHI
//	detector = new GoodFeaturesToTrackDetector(DESIRED_FTRS, 0.01, 1.0, 5);
//	descriptor = new BriefDescriptorExtractor();

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
	namedWindow(pair_window_name.c_str(), CV_WINDOW_NORMAL);
	namedWindow(relative_window_name.c_str(), CV_WINDOW_NORMAL);
	/****************************/





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



	/************************/
	/** Motion Structures  **/
	/************************/
	//Projection matrix
	Mat P = Mat::eye(4,4,CV_64F);

	//Rotation matrix
	Matx33d R;
	//traslation vector
	Vec3d t;
	//Pose vector
	Vec3d pose(0,0,0);
	/************************/



//	for (int i = 1; i < database_size + 1; i++)
	for (int i = 1;; i++)
	{


		/************************/
		/**Leitura da Image *****/
		/************************/
		char img_path[100];
		sprintf(img_path, "%s%04d%s", base_path.c_str(), i, extension.c_str());
		cout << img_path << endl;

		current_image = imread(string(img_path), CV_LOAD_IMAGE_GRAYSCALE);
		if(current_image.empty()) break;
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
			for (int i=0 ; i < (int) radiusMatches.size(); i++)
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

				drawMatchesRelative(train_pts, query_pts, current_image, relative_image, inlier_mask);

//				cout << P << endl;
//
//				R = P(Range::all(),Range(0,3));
//				t = P.col(3);
//
//				double ry = asin( R(0,2) ),
//					   rx = asin( R(2,1)/cos(ry) ),
//					   rz = asin( R(1,0)/cos(ry) );
//				double cx = cos(rx),cy = cos(ry),cz = cos(rz),
//					   sx = sin(rx),sy = sin(ry),sz = sin(rz);
//
//				cout << rad2deg(rx) << ' ' << rad2deg(ry) << ' ' << rad2deg(rz) << endl << endl;
//
//				Mat C(4,4,CV_32F);
//				C.at<float>(0,0) = +cy*cz;          C.at<float>(0,1) = -cy*sz;          C.at<float>(0,2) = +sy;    C.at<float>(0,3) = t(0);
//				C.at<float>(1,0) = +sx*sy*cz+cx*sz; C.at<float>(1,1) = -sx*sy*sz+cx*cz; C.at<float>(1,2) = -sx*cy; C.at<float>(1,3) = t(1);
//				C.at<float>(2,0) = -cx*sy*cz+sx*sz; C.at<float>(2,1) = +cx*sy*sz+sx*cz; C.at<float>(2,2) = +cx*cy; C.at<float>(2,3) = t(2);
//				C.at<float>(3,0) = 0;               C.at<float>(3,1) = 0;               C.at<float>(3,2) = 0;      C.at<float>(3,3) = 1;
//
//				C = C.inv();
//
//				cout << C << endl;
//
//				R = C( Range(0,3),Range(0,3) ); R = R.t();
//				t = C( Range(0,3),Range(3,4) );
//
//				ry = asin( R(0,2) ),
//				rx = asin( R(2,1)/cos(ry) ),
//				rz = asin( R(1,0)/cos(ry) );
//
//				cout << rad2deg(rx) << ' ' << rad2deg(ry) << ' ' << rad2deg(rz) << endl;

				//			pose = R*pose+t;
			}
		}

		train_kpts = query_kpts;
		query_desc.copyTo(train_desc);
		current_image.copyTo(previous_image);

		if (!pair_image.empty())
		{
			imshow(pair_window_name.c_str(), pair_image);
		}
		if (!relative_image.empty())
		{
			imshow(relative_window_name.c_str(), relative_image);
		}

		if ((char) waitKey(0) == 'q')
			break;
	}
}
