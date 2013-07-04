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
	double value_double;

	nh.param<std::string>("FEATURE_TYPE", value_str, "SIFT");
	parameter["FEATURE_TYPE"] = Feature::getFeatureByName(value_str);

	nh.param<int>("MAX_NUM_FEATURE_PTS", value_int, 100);
	parameter["MAX_NUM_FEATURE_PTS"] = value_int;

	nh.param<double>("MATCH_RADIUS", value_double, 10.0);
	parameter["MATCH_RADIUS"] = value_double;

	ROS_DEBUG("FEATURE_TYPE = %s", value_str.c_str());
	ROS_DEBUG("MAX_NUM_FEATURE_PTS = %d", value_int);
	ROS_DEBUG("MAX_NUM_FEATURE_PTS = %f", value_double);

	return 0;
}

//////////////////////////////////////////////////////////////////////////
/**						Image Processor Class						   **/
//////////////////////////////////////////////////////////////////////////
ImageProcessor::ImageProcessor()
{
	maxNumberOfFeatures = 100;
	feature_type = SIFT;
	radius = 10.0;
}

ImageProcessor::ImageProcessor(ImageProcessorParameter param)
{

	this->maxNumberOfFeatures = 100;
	this->feature_type = SIFT;
	this->radius = 10.0;

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
	matcher = new cv::FlannBasedMatcher();
	matcher = new cv::BFMatcher(cv::NORM_L2);
}

ImageProcessor::~ImageProcessor()
{
	// TODO Auto-generated destructor stub
//	delete detector;
//	delete extractor;
}

int ImageProcessor::setting(ImageProcessorParameter param)
{
	int IP_ERR_CODE = 0; //Error Code

	maxNumberOfFeatures = param.getParameterByName<int>("MAX_NUM_FEATURE_PTS");
	feature_type = param.getParameterByName<feature_t>("FEATURE_TYPE");
	radius = param.getParameterByName<double>("MATCH_RADIUS");

	switch (feature_type)
	{
	/*
	 * GoodFeaturesToTrackDetector( int maxCorners, double qualityLevel,
	 *                           	double minDistance, int blockSize=3,
	 *                           	bool useHarrisDetector=false, double k=0.04 );
	 */
	case SHI_TOMASI:
		detector = new cv::GoodFeaturesToTrackDetector(maxNumberOfFeatures,
				0.01, 10.0, 3, false);
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

	return IP_ERR_CODE;
}

void ImageProcessor::detect_features(cv::Mat image,
		std::vector<cv::KeyPoint> &kpts)
{
	detector->detect(image, kpts);
}

void ImageProcessor::extract_features(cv::Mat image,
		std::vector<cv::KeyPoint> &features, cv::Mat &descriptors)
{
	extractor->compute(image, features, descriptors);
}

/**
 * Method match_features:
 *
 * @param queryKeyPoints
 * @param trainKeyPoints
 * @param queryDescriptors
 * @param trainDescriptors
 * @param matches
 */
void ImageProcessor::match_features(std::vector<cv::KeyPoint> queryKeyPoints,
		std::vector<cv::KeyPoint> trainKeyPoints, cv::Mat queryDescriptors,
		cv::Mat trainDescriptors, std::vector<cv::DMatch> &matches)
{
	std::vector<std::vector<cv::DMatch> > radiusMatches;
	matcher->radiusMatch(queryDescriptors, trainDescriptors, radiusMatches,
			radius);
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

int ImageProcessor::ShiTomasiCorner(cv::Ptr<cv::FeatureDetector> det,
		cv::Mat src, std::vector<cv::KeyPoint> &arrayOfFeatures)
{
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

/**
 * @brief Feature detection using either Harris or Shi-Tomasi methods
 * 	for corner detection.
 *
 * @param det
 * @param src
 * @param arrayOfFeatures
 * @return
 */
int ImageProcessor::HarrisCorner(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
		std::vector<cv::KeyPoint> &arrayOfFeatures)
{
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

/**
 * @brief Feature detection using SURF methods for corner detection.
 *
 * @param det
 * @param src
 * @param arrayOfFeatures
 * @return
 */
int ImageProcessor::ORB_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
		std::vector<cv::KeyPoint> &arrayOfFeatures)
{
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

int ImageProcessor::FAST_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
		std::vector<cv::KeyPoint> &arrayOfFeatures)
{
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

int ImageProcessor::SURF_Detector(cv::Ptr<cv::FeatureDetector> det, cv::Mat src,
		std::vector<cv::KeyPoint> &arrayOfFeatures)
{
	// Apply corner detection
	det->detect(src, arrayOfFeatures);

	return 0;
}

int ImageProcessor::draw_feature(cv::Mat inImage, cv::Mat &outImage,
		std::vector<cv::KeyPoint> kpts)
{
	cv::cvtColor(inImage, outImage, CV_GRAY2BGR);
	cv::drawKeypoints(outImage, kpts, outImage, cv::Scalar(0, 255, 0));

	return 0;
}

int ImageProcessor::draw_matches(cv::Mat inQueryImage,
		std::vector<cv::KeyPoint> query_kpts, cv::Mat inTrainImage,
		std::vector<cv::KeyPoint> train_kpts, std::vector<cv::DMatch> matches,
		cv::Mat &outPairImage, std::vector<char> mask)
{
	cv::drawMatches(inQueryImage, query_kpts, inTrainImage, train_kpts, matches,
			outPairImage, cv::Scalar::all(-1), cv::Scalar::all(-1), mask,
			cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	return 0;
}

int ImageProcessor::draw_optflow(const cv::Mat inImage, cv::Mat &outImage,
		const std::vector<cv::KeyPoint>& query,
		const std::vector<cv::KeyPoint>& train,
		std::vector<cv::DMatch>& matches, const std::vector<char>& mask =
				std::vector<char>())
{
	cv::cvtColor(inImage, outImage, CV_GRAY2BGR);
	for (uint i = 0; i < matches.size(); i++)
	{
		if (!mask.empty())
		{
			if (!mask[i])
			{
				cv::Point2d pt_new = query[matches[i].queryIdx].pt;
				cv::Point2d pt_old = train[matches[i].trainIdx].pt;

				cv::line(outImage, pt_new, pt_old, cv::Scalar(125, 125, 255),
						1);
				cv::circle(outImage, pt_new, 2, cv::Scalar(0, 0, 255), 1);
			}
			else
			{
				cv::Point2d pt_new = query[matches[i].queryIdx].pt;
				cv::Point2d pt_old = train[matches[i].trainIdx].pt;

				cv::line(outImage, pt_new, pt_old, cv::Scalar(125, 255, 125),
						1);
				cv::circle(outImage, pt_new, 2, cv::Scalar(0, 255, 0), 1);
			}
		}
		else
		{
			cv::Point2d pt_new = query[matches[i].queryIdx].pt;
			cv::Point2d pt_old = train[matches[i].trainIdx].pt;

			cv::line(outImage, pt_new, pt_old, cv::Scalar(125, 255, 125), 1);
			cv::circle(outImage, pt_new, 2, cv::Scalar(0, 255, 0), 1);
		}
	}

	return 0;
}

} /* namespace LRM */
