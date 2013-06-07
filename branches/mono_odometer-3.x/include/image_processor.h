/*
 * feature.h
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#ifndef FEATURE_H_
#define FEATURE_H_

#include "core.h"
#include "parameter.h"

namespace LRM
{

#define DEFAULT_RADIUS 10.0

class ImageProcessorParameter: public Parameter
{


public:
	ImageProcessorParameter(){}

	~ImageProcessorParameter(){}

	int parse(ros::NodeHandle nh);

};

/*
 *	Class Feature:
 *		The class Feature defines and stores the features of the image.
 */
class ImageProcessor
{
private:
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;
	cv::Ptr<cv::DescriptorMatcher> matcher;

	int maxNumberOfFeatures;
	feature_t feature_type;
	double radius;

public:
	ImageProcessor();
	ImageProcessor(ImageProcessorParameter param);
	virtual ~ImageProcessor();

	/**
	 * Method setting:
	 *
	 * @param param
	 * @return
	 */
	int setting(ImageProcessorParameter param);

	void setRadius(double radius)
	{
		this->radius = radius;
	}

	void detect_features(cv::Mat image, std::vector<cv::KeyPoint> &kpts);
	void extract_features(cv::Mat image, std::vector<cv::KeyPoint> &features,
			cv::Mat &descriptors);
	void match_features(std::vector<cv::KeyPoint> queryKeyPoints,
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

} /* namespace LRM */
#endif /* FEATURE_H_ */
