/*
 * feature.h
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#ifndef FEATURE_H_
#define FEATURE_H_

#include "core.h"

namespace LRM
{

#define DEFAULT_RADIUS 10.0

///// Types of possible features
//enum feature_t {
//	SHI_TOMASI,
//	HARRIS,
//	ORB,
//	FAST,
//	SURF,
//	SIFT,
//	NO_FEATURE
//};
//
///// Types of possible descriptors
//enum descriptor_t {
//	SSD
//};

class Descriptor;

/*
 *	Class Feature:
 *		The class Feature defines and stores the features of the image.
 */
class FeatureHandler
{
private:
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;
	cv::Ptr<cv::DescriptorMatcher> matcher;

	int maxNumberOfFeatures;
	feature_t feature_type;
	double radius;

public:
	FeatureHandler()
	{
	}
	FeatureHandler(feature_t feature_type, int maxNumberOfFeatures);
	virtual ~FeatureHandler();

	void setRadius(double radius)
	{
		this->radius = radius;
	}

	void detect(cv::Mat image, std::vector<cv::KeyPoint> &features);
	void extract(cv::Mat image, std::vector<cv::KeyPoint> &features,
			cv::Mat &descriptors);
	void match(std::vector<cv::KeyPoint> queryKeyPoints,
			std::vector<cv::KeyPoint> trainKeyPoints, cv::Mat queryDescriptors,
			cv::Mat trainDescriptors, std::vector<cv::DMatch> &matches);

	static int ShiTomasiCorner(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
			std::vector<cv::KeyPoint> &arrayOfFeatures);
	static int HarrisCorner(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
			std::vector<cv::KeyPoint> &arrayOfFeatures);
	static int ORB_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
			std::vector<cv::KeyPoint> &arrayOfFeatures);
	static int FAST_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
			std::vector<cv::KeyPoint> &arrayOfFeatures);
	static int SURF_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
			std::vector<cv::KeyPoint> &arrayOfFeatures);
};

class Descriptor
{

};

} /* namespace LRM */
#endif /* FEATURE_H_ */
