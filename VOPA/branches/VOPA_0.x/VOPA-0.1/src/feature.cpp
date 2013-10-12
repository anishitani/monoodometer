/*
 * feature.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#include "feature.h"

namespace LRM {

FeatureHandler::FeatureHandler(feature_t feature_type, int maxNumberOfFeatures) {
	this->maxNumberOfFeatures = maxNumberOfFeatures;
	this->feature_type = feature_type;
	this->radius = DEFAULT_RADIUS;

	switch (feature_type) {
	case SHI_TOMASI:
		detector = new cv::GoodFeaturesToTrackDetector(maxNumberOfFeatures, 0.01, 10.0);
		extractor = new cv::BriefDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case HARRIS:
		detector = new cv::GoodFeaturesToTrackDetector(maxNumberOfFeatures, 0.01, 10.0, 3, true);
		extractor = new cv::BriefDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case ORB:
		detector = new cv::ORB( maxNumberOfFeatures );
		extractor = new cv::OrbDescriptorExtractor( maxNumberOfFeatures );
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case FAST:
		detector = new cv::GridAdaptedFeatureDetector( new cv::FastFeatureDetector(10, true), maxNumberOfFeatures, 4, 4);
//		detector = new cv::PyramidAdaptedFeatureDetector( new cv::FastFeatureDetector(10, true) );
		extractor = new cv::OrbDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case SURF:
		detector = new cv::SurfFeatureDetector(400.0);
		extractor = new cv::SurfDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce");
		break;
	case SIFT:
		detector = new cv::SiftFeatureDetector(maxNumberOfFeatures);
		extractor = new cv::SiftDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce");
		break;
	default:
		break;
	}
//	matcher = new cv::FlannBasedMatcher();
//	matcher = new cv::BFMatcher(cv::NORM_L2);
}

FeatureHandler::~FeatureHandler() {
	// TODO Auto-generated destructor stub
//	delete detector;
//	delete extractor;
}

/**
 * Method detect:
 *
 * 	 @param[in]	f	Base frame for feature detection
 */
void FeatureHandler::detect(cv::Mat image,
		std::vector<cv::KeyPoint> &features) {
	detector->detect(image, features);
}

/**
 * Method extract:
 *
 * 	 @param[in]	f	Base frame for feature description extraction
 */
void FeatureHandler::extract(cv::Mat image, std::vector<cv::KeyPoint> &features, cv::Mat &descriptors) {
	extractor->compute(image, features, descriptors);
}

/**
 * Method match:
 *
 * 	 @param[in]	f	Base frame for feature description extraction
 */
void FeatureHandler::match( cv::Mat queryDescriptors,
							cv::Mat trainDescriptors,
							std::vector<cv::KeyPoint> queryKeyPoints,
							std::vector<cv::KeyPoint> trainKeyPoints,
							std::vector<cv::DMatch> &matches) {
	std::vector<cv::DMatch> temp_matches;
	matcher->match( queryDescriptors,trainDescriptors,temp_matches );

	double max_dist = 0;
	double min_dist = 20;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < queryDescriptors.rows; i++) {
		double dist = temp_matches[i].distance;
		if(dist<1) continue;
		if (dist < min_dist)
			min_dist = dist;
		if (dist > max_dist)
			max_dist = dist;
	}

	matches.clear();
	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
	//-- PS.- radiusMatch can also be used here.
	for (uint i = 0; i < temp_matches.size(); i++) {
		cv::Point2f pt_query = queryKeyPoints[ temp_matches[i].queryIdx ].pt;
		cv::Point2f pt_train = trainKeyPoints[ temp_matches[i].trainIdx ].pt;
		if( (pt_query.x-pt_train.x)*(pt_query.x-pt_train.x)+(pt_query.y-pt_train.y)*(pt_query.y-pt_train.y) > radius*radius)
			continue;
		if (temp_matches[i].distance < 60 ) {
			matches.push_back(temp_matches[i]);
		}
	}

//	matcher->match( queryDescriptors,trainDescriptors,matches );
}

/**
 * Method ShiTomasiCorner:
 * 		Feature detection using either Harris or Shi-Tomasi methods
 * 	for corner detection.
 *
 * 	 @param[in]	src				Input image
 * 	 @param[in]	arrayOfFeatures	Array of detected features
 */
int FeatureHandler::ShiTomasiCorner(cv::Ptr<cv::FeatureDetector> det, cv::Mat src, std::vector<cv::KeyPoint> &arrayOfFeatures) {
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

/**
 * Method HarrisCorner:
 * 		Feature detection using either Harris or Shi-Tomasi methods
 * 	for corner detection.
 *
 * 	 @param[in]	src				Input image
 * 	 @param[in]	arrayOfFeatures	Array of detected features
 */
int FeatureHandler::HarrisCorner(cv::Ptr<cv::FeatureDetector> det, cv::Mat src, std::vector<cv::KeyPoint> &arrayOfFeatures) {
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

/**
 * Method ORB:
 * 		Feature detection using SURF methods for corner detection.
 *
 * 	 @param[in]	src				Input image
 * 	 @param[in]	arrayOfFeatures	Array of detected features
 */
int FeatureHandler::ORB_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
		std::vector<cv::KeyPoint> &arrayOfFeatures) {
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

/**
 * Method FAST:
 * 		Feature detection using FAST methods for corner detection.
 *
 * 	 @param[in]	src				Input image
 * 	 @param[in]	arrayOfFeatures	Array of detected features
 */
int FeatureHandler::FAST_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src, std::vector<cv::KeyPoint> &arrayOfFeatures) {
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

/**
 * Method SURF:
 * 		Feature detection using SURF methods for corner detection.
 *
 * 	 @param[in]	src				Input image
 * 	 @param[in]	arrayOfFeatures	Array of detected features
 */
int FeatureHandler::SURF_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src, std::vector<cv::KeyPoint> &arrayOfFeatures) {
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

} /* namespace LRM */
