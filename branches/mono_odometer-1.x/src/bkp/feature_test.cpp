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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "feature.cpp"

using namespace LRM;

image_transport::Subscriber sub;
image_transport::Publisher pub;

void ImageCb(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(msg, "bgr8");

	img_ptr->image = cv::Mat( msg->height, msg->width, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step );

	cv::Mat img_gray;
	cv::cvtColor( img_ptr->image,img_gray,CV_BGR2GRAY );

	/// Parameters for Shi-Tomasi algorithm
	std::vector<cv::Point2f> corners;
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;
	int maxCorners = 200;

	/// Apply corner detection
	cv::goodFeaturesToTrack( img_gray,
							 corners,
							 maxCorners,
							 qualityLevel,
							 minDistance,
							 cv::Mat(),
							 blockSize,
							 useHarrisDetector,
							 k );

	/// Draw corners detected
	int r = 4;
	for( uint8_t i = 0; i < corners.size(); i++ )
		cv::circle( img_ptr->image, corners[i], r, cv::Scalar(0, 255, 0), -1, 8, 0 );

//	img_ptr->image = dst_norm_scaled;

	pub.publish( img_ptr->toImageMsg() );
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
	pub = it.advertise( "/camera/image_processed", 1 );

	ros::spin();
}

