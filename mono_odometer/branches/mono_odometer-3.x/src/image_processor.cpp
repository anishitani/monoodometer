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

	nh.param < std::string > ("FEATURE_TYPE", value_str, "SIFT");
	parameter["FEATURE_TYPE"] = Feature::getFeatureByName(value_str);

	nh.param<int>("MAX_NUM_FEATURE_PTS", value_int, 1000);
	parameter["MAX_NUM_FEATURE_PTS"] = value_int;

	nh.param<double>("MATCH_RADIUS", value_double, 10.0);
	parameter["MATCH_RADIUS"] = value_double;

	nh.param<double>("MATCH_L1_ERROR", value_double, 10.0);
	parameter["MATCH_L1_ERROR"] = value_double;

	ROS_DEBUG("FEATURE_TYPE = %s", value_str.c_str());
	ROS_DEBUG("MAX_NUM_FEATURE_PTS = %d", value_int);
	ROS_DEBUG("MATCH_RADIUS = %f", value_double);
	ROS_DEBUG("MATCH_L1_ERROR = %f", value_double);

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
	match_error = 10.0;
}

ImageProcessor::ImageProcessor(ImageProcessorParameter param)
{

	this->maxNumberOfFeatures = 100;
	this->feature_type = SIFT;
	this->radius = 10.0;
	match_error = 10.0;

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
//		detector = new cv::GridAdaptedFeatureDetector(
//				new cv::FastFeatureDetector(10, true), maxNumberOfFeatures, 4,
//				4);
//		detector = new cv::PyramidAdaptedFeatureDetector( new cv::FastFeatureDetector(10, true) );
		detector = new cv::FastFeatureDetector();
		extractor = new cv::OrbDescriptorExtractor();
		matcher = cv::DescriptorMatcher::create("BruteForce");
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
//		detector = new cv::GridAdaptedFeatureDetector(
//				new cv::FastFeatureDetector(10, true), maxNumberOfFeatures, 4,
//				4);
//		detector = new cv::PyramidAdaptedFeatureDetector( new cv::FastFeatureDetector(10, true) );
		detector = new cv::FastFeatureDetector();
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
	ROS_DEBUG("Number of detected points %d", kpts.size());
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
void ImageProcessor::match_features(cv::Mat queryDescriptors,
		cv::Mat trainDescriptors, std::vector<cv::DMatch> &matches)
{
	std::vector < std::vector<cv::DMatch> > _matches;
	matcher->radiusMatch(queryDescriptors, trainDescriptors, _matches, radius);

	matches.clear();
	for (int i = 0; i < (int) _matches.size(); i++)
	{
		if (_matches[i].size())
		{
			float dist = radius;
			int pos = 0;
			for (int j = 0; j < (int) _matches[i].size(); j++)
			{
				if (dist > _matches[i][j].distance)
				{
					dist = _matches[i][j].distance;
					pos = j;
				}
			}
			if (dist > radius)
				continue;
			matches.push_back(_matches[i][pos]);
		}
	}
}

int ImageProcessor::match_features_optflow(cv::Mat queryImg, cv::Mat trainImg,
		std::vector<cv::KeyPoint> &query_kpts,
		std::vector<cv::KeyPoint> &train_kpts, std::vector<cv::DMatch> &matches)
{
	std::vector<cv::Point2f> query_pts, train_pts;
	convertKeypointToPoint(query_kpts, query_pts);
	convertKeypointToPoint(train_kpts, train_pts);

	std::vector < cv::Point2f > query_optflow_pts(train_pts.size());

	std::vector < uchar > status;
	std::vector<float> error;

	cv::calcOpticalFlowPyrLK(trainImg, queryImg, train_pts, query_optflow_pts,
			status, error);
	for (uint i = 0; i < status.size(); i++)
	{
		if (status[i] && error[i] > match_error)
			status[i] = 0;
	}

	cv::Mat query_optflow_desc = cv::Mat(query_optflow_pts).reshape(1,
			query_optflow_pts.size());
	cv::Mat query_desc = cv::Mat(query_pts).reshape(1, query_pts.size());
	std::vector < std::vector<cv::DMatch> > _matches;

	cv::BFMatcher _matcher(cv::NORM_L2);

	/**
	 * @todo The matcher type definition should be revised later.
	 */
	_matcher.radiusMatch(query_optflow_desc, query_desc, _matches, 2.0f);

	/**
	 * @todo Think of a better solution then the flag.
	 */
	std::vector<bool> used_query(query_pts.size(), false);

	matches.clear();

	for (uint i = 0; i < _matches.size(); i++)
	{
		if (_matches[i].size() == 0)
		{
			/*
			 * If there's no feature where the optical flow was expecting a feature.
			 */
			status[i] = 0;
			continue;
		}
		else if (_matches[i].size() > 1)
		{
			// 2 neighbors – check how close they are
			double ratio = _matches[i][0].distance / _matches[i][1].distance;
			if (ratio < 0.7)
			{
				/*
				 * If there're more then 1 features and the two closest are
				 * closer than a ratio (0.7 in this case).
				 */
				status[i] = 0;
				continue;
			}
		}
		if (status[i] && !used_query[_matches[i][0].trainIdx])
		{
			/*
			 * The radius matcher related the points estimated by the optical flow
			 * with the points detected by the feature detector. But it should be
			 * inverted to fit the relation train query feature.
			 */
			cv::DMatch match;

			match.trainIdx = _matches[i][0].queryIdx;
			match.queryIdx = _matches[i][0].trainIdx;
			matches.push_back(match);
			used_query[_matches[i][0].trainIdx] = true;
		}

	}

	return 0;
}

int ImageProcessor::convertKeypointToPoint(std::vector<cv::KeyPoint> kpts,
		std::vector<cv::Point2f> &pts)
{
	pts.clear();
	for (uint i = 0; i < kpts.size(); i++)
	{
		pts.push_back(kpts[i].pt);
	}

	return 0;
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

//int ImageProcessor::draw_optflow(const cv::Mat inImage, cv::Mat &outImage,
//		const std::vector<cv::KeyPoint>& query,
//		const std::vector<cv::KeyPoint>& train,
//		std::vector<cv::DMatch>& matches, const std::vector<char> mask =
//				std::vector<char>())
//{
//	cv::cvtColor(inImage, outImage, CV_GRAY2BGR);
//	for (uint i = 0; i < matches.size(); i++)
//	{
//		if (!mask.empty())
//		{
//			if (!mask[i])
//			{
//				cv::Point2d pt_new = query[matches[i].queryIdx].pt;
//				cv::Point2d pt_old = train[matches[i].trainIdx].pt;
//
//				cv::line(outImage, pt_new, pt_old, cv::Scalar(125, 125, 255),
//						1);
//				cv::circle(outImage, pt_new, 2, cv::Scalar(0, 0, 255), 1);
//			}
//			else
//			{
//				cv::Point2d pt_new = query[matches[i].queryIdx].pt;
//				cv::Point2d pt_old = train[matches[i].trainIdx].pt;
//
//				cv::line(outImage, pt_new, pt_old, cv::Scalar(125, 255, 125),
//						1);
//				cv::circle(outImage, pt_new, 2, cv::Scalar(0, 255, 0), 1);
//			}
//		}
//		else
//		{
//			cv::Point2d pt_new = query[matches[i].queryIdx].pt;
//			cv::Point2d pt_old = train[matches[i].trainIdx].pt;
//
//			cv::line(outImage, pt_new, pt_old, cv::Scalar(125, 255, 125), 1);
//			cv::circle(outImage, pt_new, 2, cv::Scalar(0, 255, 0), 1);
//		}
//	}
//
//	return 0;
//}

int ImageProcessor::draw_optflow(const cv::Mat inImage, cv::Mat &outImage,
		const std::vector<cv::KeyPoint>& query,
		const std::vector<cv::KeyPoint>& train,
		std::vector<cv::DMatch>& matches, const std::vector<char> mask =
				std::vector<char>())
{
	cv::cvtColor(inImage, outImage, CV_GRAY2BGR);
	int line_thickness = 1;
	for (uint i = 0; i < matches.size(); i++)
	{
		cv::Point pt_new = query[matches[i].queryIdx].pt;
		cv::Point pt_old = train[matches[i].trainIdx].pt;

		double angle = atan2((double) pt_old.y - pt_new.y,
				(double) pt_old.x - pt_new.x);

		double hypotenuse = sqrt(
				(double) (pt_old.y - pt_new.y) * (pt_old.y - pt_new.y)
						+ (double) (pt_old.x - pt_new.x)
								* (pt_old.x - pt_new.x));

		if (hypotenuse < 1.0)
			continue;

		if (!mask.empty() && !mask[i])
		{
//			cv::line(outImage, pt_new, pt_old, cv::Scalar(125, 125, 255), 1);
//			cv::circle(outImage, pt_new, 2, cv::Scalar(0, 0, 255), 1);

			// Here we lengthen the arrow by a factor of three.
			pt_new.x = (int) (pt_old.x - 1 * hypotenuse * cos(angle));
			pt_new.y = (int) (pt_old.y - 1 * hypotenuse * sin(angle));

			// Now we draw the main line of the arrow.
			cv::line(outImage, pt_old, pt_new, cv::Scalar(0,0,255), line_thickness);

			// Now draw the tips of the arrow. I do some scaling so that the
			// tips look proportional to the main line of the arrow.

			pt_old.x = (int) (pt_new.x + 9 * cos(angle + CV_PI / 4));
			pt_old.y = (int) (pt_new.y + 9 * sin(angle + CV_PI / 4));
			cv::line(outImage, pt_old, pt_new, cv::Scalar(0,0,255), line_thickness);

			pt_old.x = (int) (pt_new.x + 9 * cos(angle - CV_PI / 4));
			pt_old.y = (int) (pt_new.y + 9 * sin(angle - CV_PI / 4));
			cv::line(outImage, pt_old, pt_new, cv::Scalar(0,0,255), line_thickness);

			continue;
		}

		// Here we lengthen the arrow by a factor of three.
		pt_new.x = (int) (pt_old.x - 3 * hypotenuse * cos(angle));
		pt_new.y = (int) (pt_old.y - 3 * hypotenuse * sin(angle));

		// Now we draw the main line of the arrow.
		cv::line(outImage, pt_old, pt_new, cv::Scalar(0, 255, 0), line_thickness);

		// Now draw the tips of the arrow. I do some scaling so that the
		// tips look proportional to the main line of the arrow.

		pt_old.x = (int) (pt_new.x + 9 * cos(angle + CV_PI / 4));
		pt_old.y = (int) (pt_new.y + 9 * sin(angle + CV_PI / 4));
		cv::line(outImage, pt_old, pt_new, cv::Scalar(0, 255, 0), line_thickness);

		pt_old.x = (int) (pt_new.x + 9 * cos(angle - CV_PI / 4));
		pt_old.y = (int) (pt_new.y + 9 * sin(angle - CV_PI / 4));
		cv::line(outImage, pt_old, pt_new, cv::Scalar(0, 255, 0), line_thickness);
	}

	return 0;
}

} /* namespace LRM */
