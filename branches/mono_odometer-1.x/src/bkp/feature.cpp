/*
 * feature.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#include "feature.h"

namespace LRM {

/**
 * FeatureHandler constructor:
 *
 * 	 @param[in]	feature_type		Type of feature used.
 * 	 @param[in]	maxNumberOfFeatures	Maximum number of features the detector may detect.
 */
FeatureHandler::FeatureHandler( feature_t feature_type, int maxNumberOfFeatures ) {
	this->maxNumberOfFeatures = maxNumberOfFeatures;
	this->feature_type = feature_type;
	radius = DEFAULT_RADIUS;

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
		extractor = new cv::BriefDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case SURF:
		detector = new cv::SurfFeatureDetector(600.0);
		extractor = new cv::SurfDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("FlannBased");
		break;
	case SIFT:
		detector = new cv::SiftFeatureDetector(maxNumberOfFeatures);
		extractor = new cv::SiftDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("FlannBased");
		break;
	default:
		break;
	}
}

FeatureHandler::~FeatureHandler() {
	// TODO Auto-generated destructor stub
//	delete detector;
//	delete extractor;
}

/**
 * Method detect:
 * 		Detects the image features.
 *
 * 	 @param[in]	 image		Current frame image.
 * 	 @param[out] features	Features detected in the current frame image.
 */
void FeatureHandler::detect(cv::Mat image, std::vector<cv::KeyPoint> &features) {
	detector->detect(image, features);
}

/**
 * Method extract:
 * 		Extracts the descriptors from the features detected.
 *
 * 	 @param[in]	 image			Current frame image.
 * 	 @param[in]  features		Features detected in the current frame image.
 * 	 @param[out] descriptors	Descriptors extracted from the current frame features.
 */
void FeatureHandler::extract(cv::Mat image, std::vector<cv::KeyPoint> features, cv::Mat &descriptors) {
	extractor->compute(image, features, descriptors);
}

/**
 * Method match:
 * 		Matches the descriptors from the current and the last frames.
 *
 * 	 @param[out] queryDescriptors	Descriptors extracted from the current frame features.
 * 	 @param[out] trainDescriptors	Descriptors extracted from the last frame features.
 * 	 @param[out] matches			Matches between the current and the last frames.
 */
//void FeatureHandler::match(cv::Mat queryDescriptors, cv::Mat trainDescriptors, std::vector<cv::DMatch> &matches) {
//	std::vector<cv::DMatch> temp_matches;
//	matcher->match( queryDescriptors,trainDescriptors,temp_matches );
//
//	matches.clear();
//	for ( int i = 0; i < queryDescriptors.rows; i++ ) {
//		if ( temp_matches[i].distance < radius ) {
//			ROS_DEBUG("Minimum distance: %f - Matches distance: %f",radius, temp_matches[i].distance);
//			matches.push_back( temp_matches[i] );
//		}
//	}
//}

void FeatureHandler::match( cv::Mat queryDescriptors, cv::Mat trainDescriptors,
		std::vector<cv::KeyPoint> queryFeatures, std::vector<cv::KeyPoint> trainFeatures,
		std::vector<cv::DMatch> &matches )
{
	std::vector<cv::DMatch> temp_matches;
	matcher->match( queryDescriptors,trainDescriptors,temp_matches );

//	matches.clear();
//	matches = temp_matches;

//	for ( uint i = 0; i < temp_matches.size(); i++ ) {
//		if( temp_matches[i].distance > radius ) continue;
//		cv::Point2f pt_query(queryFeatures[ temp_matches[i].queryIdx ].pt);
//		cv::Point2f pt_train(trainFeatures[ temp_matches[i].trainIdx ].pt);
//		if ( (pt_query.x-pt_train.x)*(pt_query.x-pt_train.x)+(pt_query.y-pt_train.y)*(pt_query.y-pt_train.y) < radius*radius ) {
//			ROS_DEBUG("Minimum distance: %f - Matches distance: %f",radius, temp_matches[i].distance);
//			matches.push_back( temp_matches[i] );
//		}
//	}

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
	for (int i = 0; i < queryDescriptors.rows; i++) {
		if (temp_matches[i].distance < 10 * min_dist /*&& temp_matches[i].distance > 1 * min_dist*/ ) {
			matches.push_back(temp_matches[i]);
		}
	}
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
