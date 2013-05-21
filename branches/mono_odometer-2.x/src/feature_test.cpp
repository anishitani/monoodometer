/*
 * feature_test.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <cv_bridge/cv_bridge.h>

#include "feature.cpp"

using namespace LRM;

image_transport::Subscriber sub;
image_transport::Publisher corner_pub;
image_transport::Publisher sift_pub;

void ImageCb(const sensor_msgs::ImageConstPtr& msg) {

	cv::Ptr<cv::FeatureDetector> corner_detector,sift_detector;


	//SIFT
	sift_detector = new cv::SiftFeatureDetector();
	//SURF
//	detector = new SurfFeatureDetector(1000);
	//ORB
//	detector = new cv::ORB( DESIRED_FTRS );
	//FAST
//	detector = new cv::GridAdaptedFeatureDetector( new cv::FastFeatureDetector(10, true), DESIRED_FTRS, 4, 4);
//	detector = new cv::PyramidAdaptedFeatureDetector( new cv::FastFeatureDetector(5, true) );
	//SHI
	corner_detector = new cv::GoodFeaturesToTrackDetector(1000, 0.01, 1.0, 5);

	// Feature points
	std::vector<cv::KeyPoint> corner_kpts,sift_kpts;

	cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	cv_bridge::CvImage corner_img_ptr;
	cv_bridge::CvImage sift_img_ptr;


	corner_img_ptr.image = cv::Mat( msg->height, msg->width, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step );
	corner_img_ptr.encoding = "bgr8";
	sift_img_ptr.image = cv::Mat( msg->height, msg->width, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step );
	sift_img_ptr.encoding = "bgr8";

	cv::Mat img_gray;
	cv::cvtColor( img_ptr->image,img_gray,CV_BGR2GRAY );

	corner_detector->detect(img_gray, corner_kpts);
	cv::drawKeypoints(corner_img_ptr.image, corner_kpts, corner_img_ptr.image,
			cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	corner_pub.publish( corner_img_ptr.toImageMsg() );

	sift_detector->detect(img_gray, sift_kpts);
	cv::drawKeypoints(sift_img_ptr.image, sift_kpts, sift_img_ptr.image,
			cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	sift_pub.publish( sift_img_ptr.toImageMsg() );
}

/**
 *	main()
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "mono_odometer");
	std::string node_name = ros::this_node::getName();

	ros::NodeHandle nh_(node_name);

	image_transport::ImageTransport it( nh_ );

	sub = it.subscribe( "/camera/image", 1,ImageCb );
	corner_pub = it.advertise( "/feature_test/image/corner", 1 );
	sift_pub = it.advertise( "/feature_test/image/sift", 1 );

	ros::spin();
}

