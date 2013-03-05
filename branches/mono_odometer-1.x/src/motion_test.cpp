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

#include "motion.h"

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

//Converts key points to matching indices
//void keypoints2matches(
//		vector<KeyPoint>& train, vector<KeyPoint>& query,
//		const std::vector<cv::Point2f>& pts_train, const std::vector<Point2f>& pts_query,
//		std::vector<cv::DMatch>& matches, vector<unsigned char> mask)
//{
//	std::vector<cv::DMatch> valid_matches = new std::vector<cv::DMatch>();
//
//	int mask_size = countNonZero(Mat(mask));
//	train.clear();
//	query.clear();
//	train.reserve(mask_size);
//	query.reserve(mask_size);
//
//	size_t i = 0;
//
//	for (; i < matches.size(); i++)
//	{
//		if((int) !mask[i]) continue;
//
//		const DMatch & dmatch = matches[i];
//
//		valid_matches.push_back(matches[i]);
//
//		query.push_back(query[dmatch.queryIdx]);
//		train.push_back(train[dmatch.trainIdx]);
//	}
//	matches.clear();
//	matches = & vector<DMatch>(valid_matches);
//}

void estimate_motion(vector<Point2f> train_pts, vector<Point2f> query_pts, Mat K, Mat &P)
{
	LRM::MotionEstimator motion;

	Mat Tp, Tc;
	vector<Point2f> norm_train_pts(train_pts.size());
	vector<Point2f> norm_query_pts(query_pts.size());

	if(!motion.feature_point_normalization(train_pts, query_pts, norm_train_pts,
			norm_query_pts, Tp, Tc))
		return;

	Mat F = motion.compute_F_matrix(norm_train_pts, norm_query_pts);

	Mat E = K.t()*Tc.t()*F*Tp*K;

	SVD svd(E);
	svd.w.at<double>(2) = 0;
	E = svd.u*cv::Mat().diag(svd.w)*svd.vt;

	cv::Mat R, t;

	P = motion.compute_Rt(E, K,norm_train_pts, norm_query_pts, R, t);

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

	nh.param<std::string>("base_path",base_path,"/home/nishitani/Dropbox/usp/ros/mono_odometer/dataset");
	nh.param<std::string>("extension",extension,".jpg");
	nh.param<int>("database_size",database_size,1);


	/****************************/
	/** Feature and Descriptor **/
	/****************************/
	const int DESIRED_FTRS = 500;				// number of used features
//	SiftFeatureDetector detector(DESIRED_FTRS);	// Feature detector
//	SiftDescriptorExtractor descriptor(64);		// Extractor of features descriptors
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> descriptor;
	BFMatcher desc_matcher(NORM_L2);			// Matching rule
//	DescriptorMatcher desc_matcher;

	detector = new SiftFeatureDetector(DESIRED_FTRS);
	descriptor = new SiftDescriptorExtractor( 128 );

	vector<Point2f> train_pts, query_pts;		// Feature points
	vector<KeyPoint> train_kpts, query_kpts;	// Keypoints
	Mat train_desc, query_desc;					// Descriptors
	vector<DMatch> matches;						// Matches
	vector< vector<DMatch> > radiusMatches;
	vector<unsigned char> match_mask;			// Mask of near matches
	/****************************/

	/****************************/
	/** Images ******************/
	/****************************/
	Mat frame,
		output_image,
		current_image,
		previous_image;

	string window_name("Matches");
	namedWindow(window_name.c_str(),CV_WINDOW_NORMAL);
	/****************************/


	Mat H_prev = Mat::eye(3, 3, CV_32FC1);




	/************************/
	/** Calibration Matrix **/
	/************************/
//	Mat K;
//	FileStorage calib_data;
//	calib_data.open( "/home/nishitani/Dropbox/usp/ros/mono_odometer/calib/camera_left.yaml", cv::FileStorage::READ );
//	calib_data[ "cameraMatrix" ] >> K;
	/************************/





//	Vec3f pose(0,0,0);

	for (int i = 1; i < database_size+1; i++)
	{
		char img_path[100];
		sprintf(img_path,"%s%04d%s",base_path.c_str(),i,extension.c_str());
		cout << img_path << endl;

		current_image = imread( string(img_path), CV_LOAD_IMAGE_GRAYSCALE );
//		cvtColor(frame,current_image,CV_BGR2GRAY);

		detector->detect(current_image, query_kpts);
		descriptor->compute(current_image, query_kpts, query_desc);
//
//		Mat P;
//		Matx33f R;
//		Vec3f t;
//
		if (!train_desc.empty())
		{
			/************************/
			/** Calcula os matches **/
			/************************/
//			desc_matcher.match(query_desc, train_desc, matches);
			desc_matcher.radiusMatch(train_desc, query_desc, radiusMatches, 100.0);

//			for(int i=0 ; i<radiusMatches.size() ; i++)
//			cout << radiusMatches[i].size() << endl;

			matches.clear();
			for(int i=0 ; i< (int)radiusMatches.size() ; i++){
				if(radiusMatches[i].size()){
					float dist = 10000.0;
					int pos=0;
					for(int j=0 ; j < (int)radiusMatches[i].size() ; j++){
						if(dist>radiusMatches[i][j].distance){
							dist = radiusMatches[i][j].distance;
							pos = j;
						}
					}
					matches.push_back(radiusMatches[i][pos]);
				}
			}
			/************************/

//			matches2points(train_kpts, query_kpts, matches, train_pts,query_pts);
			if (matches.size() > 5)
			{
//				Mat H = findHomography(train_pts, query_pts, RANSAC, 4, match_mask);

//				std::cout << matches.size() << std::endl;
//				keypoints2matches(train_kpts,query_kpts,train_pts,query_pts,matches,match_mask);
//				std::cout << matches.size() << std::endl << std::endl;
//				if (countNonZero(Mat(match_mask)) > 15)
//				{
//					H_prev = H;
//				}
//				else
//					resetH(H_prev);
			}
			drawMatches(previous_image, train_kpts, current_image, query_kpts, matches, output_image,
									Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//			else
//				resetH(H_prev);

//			estimate_motion(train_pts,query_pts,Mat().eye(3,3,CV_64F),P);
//
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
		else
		{
//			H_prev = Mat::eye(3, 3, CV_32FC1);
		}


		train_kpts = query_kpts;
		query_desc.copyTo(train_desc);
		current_image.copyTo(previous_image);

		if(!output_image.empty())
			imshow(window_name.c_str(), output_image);

		if( (char)waitKey(0)=='q' ) break;
	}
}
