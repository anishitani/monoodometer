#ifndef FEATURE_H_
#define FEATURE_H_

#include "core.h"
#include "parameter.h"

namespace LRM
{

/**
 *  List of parameter regarding ROS configuration for the launch file.
 *  	- @b FEATURE_TYPE: Type of feature to be detected. (@b Default: SIFT)
 *  	- @b MAX_NUM_FEATURE_PTS: Maximum number of features to be detected. (@b Default: 100)
 *  	- @b MATCH_RADIUS: Radius used as a filter for the feature match. (@b Default: 10.0)
 */

class ImageProcessorParameter: public Parameter
{

public:
	ImageProcessorParameter()
	{
	}

	~ImageProcessorParameter()
	{
	}

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
	double match_error;

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

//	void setRadius(double radius)
//	{
//		this->radius = radius;
//	}

	void detect_features(cv::Mat image, std::vector<cv::KeyPoint> &kpts);
	void extract_features(cv::Mat image, std::vector<cv::KeyPoint> &features,
			cv::Mat &descriptors);
	void match_features(cv::Mat queryDescriptors, cv::Mat trainDescriptors,
			std::vector<cv::DMatch> &matches);

	/**
	 * @brief Based on the matcher described by Daniel Lelis Baggio in Mastering OpenCV
	 * with Practical Computer Vision Projects.
	 * @param queryImg
	 * @param trainImg
	 * @param query_kpts
	 * @param train_kpts
	 * @param matches
	 * @return
	 */
	int match_features_optflow(cv::Mat queryImg, cv::Mat trainImg,
			std::vector<cv::KeyPoint> &query_kpts,
			std::vector<cv::KeyPoint> &train_kpts,
			std::vector<cv::DMatch> &matches);

	/**
	 * @todo Allow the type of point choose. (Point2d, Point2f,etc...)
	 * @brief Converts a KeyPoint structure into a Point structure.
	 * @param kpts
	 * @param pts
	 * @return
	 */
	int convertKeypointToPoint(std::vector<cv::KeyPoint> kpts, std::vector<cv::Point2f> &pts);

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

	static int draw_feature(cv::Mat inImage, cv::Mat &outImage,
			std::vector<cv::KeyPoint> kpts);
	static int draw_matches(cv::Mat inQueryImage,
			std::vector<cv::KeyPoint> query_kpts, cv::Mat inTrainImage,
			std::vector<cv::KeyPoint> train_kpts,
			std::vector<cv::DMatch> matches, cv::Mat &outPairImage,
			std::vector<char> mask);
	static int draw_optflow(const cv::Mat inImage, cv::Mat &outImage,
			const std::vector<cv::KeyPoint>& query,
			const std::vector<cv::KeyPoint>& train,
			std::vector<cv::DMatch>& matches, const std::vector<char> mask);
//	static int draw_optflow(const cv::Mat inImage, cv::Mat &outImage,
//			const std::vector<cv::Point2d>& query,
//			const std::vector<cv::Point2d>& train);
};

} /* namespace LRM */
#endif /* FEATURE_H_ */
