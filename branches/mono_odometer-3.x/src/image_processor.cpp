/*
 * feature.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#include "image_processor.h"

namespace LRM
{

//////////////////////////////////////////////////////////////////////////
/**				Image Processor Parameter Class						   **/
//////////////////////////////////////////////////////////////////////////
int ImageProcessorParameter::parse(ros::NodeHandle nh)
{
	std::string value_str;
	int value_int;

//	nh.param<std::string>("INPUT_IMAGE_TOPIC", value_str, "/camera/image");
//	parameter["INPUT_IMAGE_TOPIC"] = value_str;
//
//	ROS_DEBUG(
//			"INPUT_IMAGE_TOPIC = %s", boost::any_cast<std::string>(parameter["INPUT_IMAGE_TOPIC"]).c_str());

	return 0;
}


ImageProcessor::ImageProcessor()
{
}

ImageProcessor::ImageProcessor(ImageProcessorParameter param)
{

	this->maxNumberOfFeatures = maxNumberOfFeatures;
	this->feature_type = feature_type;
	this->radius = DEFAULT_RADIUS;

	switch (feature_type)
	{
	case SHI_TOMASI:
		detector = new cv::GoodFeaturesToTrackDetector(maxNumberOfFeatures,
				0.01, 10.0);
		extractor = new cv::BriefDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case HARRIS:
		detector = new cv::GoodFeaturesToTrackDetector(maxNumberOfFeatures,
				0.01, 10.0, 3, true);
		extractor = new cv::BriefDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case ORB:
		detector = new cv::ORB(maxNumberOfFeatures);
		extractor = new cv::OrbDescriptorExtractor(maxNumberOfFeatures);
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case FAST:
		detector = new cv::GridAdaptedFeatureDetector(
				new cv::FastFeatureDetector(10, true), maxNumberOfFeatures, 4,
				4);
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
		detector = new cv::SiftFeatureDetector();
		extractor = new cv::SiftDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce");
		break;
	default:
		break;
	}
//	matcher = new cv::FlannBasedMatcher();
//	matcher = new cv::BFMatcher(cv::NORM_L2);
}

ImageProcessor::~ImageProcessor()
{
	// TODO Auto-generated destructor stub
//	delete detector;
//	delete extractor;
}

int ImageProcessor::setting(ImageProcessorParameter param)
{
	int err_code = 0;

	maxNumberOfFeatures = param.getParameterByName<int>("MAX_NUM_FEATURE_PTS");
	feature_type = param.getParameterByName<feature_t>("FEATURE_TYPE");
	radius = param.getParameterByName<double>("FEATURE_MATCH_RADIUS");

	switch (feature_type)
	{
	case SHI_TOMASI:
		detector = new cv::GoodFeaturesToTrackDetector(maxNumberOfFeatures,
				0.01, 10.0);
		extractor = new cv::BriefDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case HARRIS:
		detector = new cv::GoodFeaturesToTrackDetector(maxNumberOfFeatures,
				0.01, 10.0, 3, true);
		extractor = new cv::BriefDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case ORB:
		detector = new cv::ORB(maxNumberOfFeatures);
		extractor = new cv::OrbDescriptorExtractor(maxNumberOfFeatures);
		matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case FAST:
		detector = new cv::GridAdaptedFeatureDetector(
				new cv::FastFeatureDetector(10, true), maxNumberOfFeatures, 4,
				4);
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
		detector = new cv::SiftFeatureDetector();
		extractor = new cv::SiftDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce");
		break;
	default:
		break;
	}

	return err_code;
}

/**
 * Method detect:
 *
 * 	 @param[in]	f	Base frame for feature detection
 */
void ImageProcessor::detect_features(cv::Mat image,
		std::vector<cv::KeyPoint> &kpts)
{
	detector->detect(image, kpts);
}

/**
 * Method extract:
 *
 * 	 @param[in]	f	Base frame for feature description extraction
 */
void ImageProcessor::extract_features(cv::Mat image,
		std::vector<cv::KeyPoint> &features, cv::Mat &descriptors)
{
	extractor->compute(image, features, descriptors);
}

/**
 * Method match:
 *
 * 	 @param[in]	f	Base frame for feature description extraction
 */
void ImageProcessor::match_features(std::vector<cv::KeyPoint> queryKeyPoints,
		std::vector<cv::KeyPoint> trainKeyPoints, cv::Mat queryDescriptors,
		cv::Mat trainDescriptors, std::vector<cv::DMatch> &matches)
{
	std::vector<std::vector<cv::DMatch> > radiusMatches;
	matcher->radiusMatch(queryDescriptors, trainDescriptors, radiusMatches,
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
}

/**
 * Method ShiTomasiCorner:
 * 		Feature detection using either Harris or Shi-Tomasi methods
 * 	for corner detection.
 *
 * 	 @param[in]	src				Input image
 * 	 @param[in]	arrayOfFeatures	Array of detected features
 */
int ImageProcessor::ShiTomasiCorner(cv::Ptr<cv::FeatureDetector> det,
		cv::Mat src, std::vector<cv::KeyPoint> &arrayOfFeatures)
{
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
int ImageProcessor::HarrisCorner(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
		std::vector<cv::KeyPoint> &arrayOfFeatures)
{
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
int ImageProcessor::ORB_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
		std::vector<cv::KeyPoint> &arrayOfFeatures)
{
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
int ImageProcessor::FAST_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
		std::vector<cv::KeyPoint> &arrayOfFeatures)
{
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
int ImageProcessor::SURF_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
		std::vector<cv::KeyPoint> &arrayOfFeatures)
{
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

} /* namespace LRM */
