/*
 * frame.cpp
 *
 *  Created on: Sep 25, 2012
 *      Author: nishitani
 */

#include "frame.h"

namespace LRM
{

/**
 * Frame Constructor:
 * 		Instantiates a new frame. The Frame class subscribes a frame from
 * 	 a camera and process it identifying features and descriptors. Also
 * 	 responsible for the match of the features over other frames.
 *
 * 	 @param[in]	nh	Node handle of the node of the monocular odometer.
 */
Frame::Frame(const sensor_msgs::ImageConstPtr& msg)
{
	/// TODO: Deal with different image encodings
	img_ptr = cv_bridge::toCvCopy(msg, "mono8");
	timestamp = msg->header.stamp;
}

Frame::~Frame()
{
	// TODO Auto-generated destructor stub
}

/**
 * process:
 Detects features and extracts the features descriptors.
 *
 * 	 @param[in]	param Parameters for the detector and extractor.
 */
void Frame::process(FeatureHandler fh)
{
	fh.detect(img_ptr->image, keypoints);
	fh.extract(img_ptr->image, keypoints, descriptors);
}

/**
 * match:
 * 	Matches the descriptors of the last frame with current one.
 *
 * @param fh 		Feature handler
 * @param query		Current frame
 * @param train		Last Frame
 * @param matches	Matches between frames
 */
void Frame::match(FeatureHandler fh, Frame query, Frame train,
		std::vector<cv::DMatch> &matches)
{
	fh.match(query.getKeyPoints(), train.getKeyPoints(), query.getDescriptors(),
			train.getDescriptors(), matches);
}

void Frame::motion(MotionEstimator &me, Frame query, Frame train,
		std::vector<cv::DMatch> matches, cv::Mat K, cv::Mat &P)
{
	std::vector<cv::Point2d> query_pts, train_pts;

	me.matches2points(query.getKeyPoints(), train.getKeyPoints(), matches,
			query_pts, train_pts);

	me.estimate_motion(train_pts, query_pts, matches, K, P);

//	std::cout << P << std::endl;
}

void Frame::draw(cv::Mat &outImg, Frame query, Frame train,
		std::vector<cv::DMatch> matches, OdomParam param,
		std::vector<char> mask)
{
	if (param.getDrawPair())
	{
		cv::drawMatches(query.getImage(), query.getKeyPoints(),
				train.getImage(), train.getKeyPoints(), matches, outImg,
				cv::Scalar::all(-1), cv::Scalar::all(-1), mask,
				cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	}
	if (param.getDrawKeypoints())
	{
		cv::drawKeypoints(query.getImage(), query.getKeyPoints(), outImg,
				cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	}
	if (param.getDrawTracks())
	{
		cv::cvtColor(query.getImage(), outImg, CV_GRAY2BGR);
		for (int i = 0; i < (int) matches.size(); i++)
		{
			if (!mask.empty())
			{
				if (mask[i])
				{
					//Inliers
					cv::Point2d pt_new =
							query.getKeyPoints()[matches[i].queryIdx].pt;
					cv::Point2d pt_old =
							train.getKeyPoints()[matches[i].trainIdx].pt;

					cv::line(outImg, pt_new, pt_old, cv::Scalar(125, 255, 125),
							2);
					cv::circle(outImg, pt_new, 2, cv::Scalar(0, 255, 0), 2);
				}
				else
				{

					//Outliers
					cv::Point2d pt_new =
							query.getKeyPoints()[matches[i].queryIdx].pt;
					cv::Point2d pt_old =
							train.getKeyPoints()[matches[i].trainIdx].pt;

					cv::line(outImg, pt_new, pt_old, cv::Scalar(125, 125, 255),
							2);
					cv::circle(outImg, pt_new, 2, cv::Scalar(0, 0, 255), 2);
				}
			}
			else
			{
				cv::Point2d pt_new =
						query.getKeyPoints()[matches[i].queryIdx].pt;
				cv::Point2d pt_old =
						train.getKeyPoints()[matches[i].trainIdx].pt;

				cv::line(outImg, pt_new, pt_old, cv::Scalar(125, 255, 125), 2);
				cv::circle(outImg, pt_new, 2, cv::Scalar(0, 255, 0), 2);
			}
		}
	}
}

/**
 * markFeatures:
 *		Mark the features in the image
 *
 */
//void Frame::markFeatures()
//{
//	cv::drawKeypoints(img_ptr->image, keypoints, img_ptr->image,
//			CV_RGB(0,255,0),
//			cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
//					| cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
//}
/**
 * markTracks:
 *		Mark the features track in the image
 *
 * 	 @param[in]	lastFeatList	The list of features in the last frame
 */
//void Frame::markTracks(std::vector<cv::KeyPoint> lastFeatList)
//{
//	for (uint i = 0; i < featMatches.size(); i++)
//	{
//		cv::Point2f pt_new = featList[featMatches[i].queryIdx].pt;
//		cv::Point2f pt_old = lastFeatList[featMatches[i].trainIdx].pt;
//
//		cv::line(img_ptr->image, pt_old, pt_new, cv::Scalar(0, 0, 255), 1);
//	}
//}
} /* namespace LRM */
