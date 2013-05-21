/*
 * frame.h
 *
 *  Created on: Sep 25, 2012
 *      Author: nishitani
 */

#ifndef FRAME_H_
#define FRAME_H_

#include "core.h"
#include "parameter.h"
#include "motion.h"
#include "feature.h"

namespace LRM
{

/**
 *	Class Frame:
 *		The class Frame is responsible to retain the information about a frame
 *	of the data stream. Also responsible of possible alterations on the image.
 */
class Frame
{
public:
	// Constructor and Destructor
	Frame(const sensor_msgs::ImageConstPtr& msg);
	virtual ~Frame();

	// Public methods
	void process(FeatureHandler fh);

	static void match(FeatureHandler fh, Frame query, Frame train,
			std::vector<cv::DMatch> &matches);
	static void motion(MotionEstimator &me, Frame query, Frame train,
			std::vector<cv::DMatch> matches, cv::Mat K, cv::Mat &P);
	static void draw(cv::Mat &outImg, Frame f1, Frame f2,
			std::vector<cv::DMatch> matches, OdomParam param,
			std::vector<char> mask = std::vector<char>());

	//gets
	sensor_msgs::ImagePtr getImageMsg()
	{
		return img_ptr->toImageMsg();
	}
	int getNumberOfFeatures()
	{
		return keypoints.size();
	}
	std::vector<cv::KeyPoint> getKeyPoints()
	{
		return keypoints;
	}
	cv::Mat getDescriptors()
	{
		return descriptors;
	}
	ros::Time getTimestamp()
	{
		return timestamp;
	}
	cv::Mat getImage()
	{
		return img_ptr->image;
	}

private:
	std::vector<cv::KeyPoint> keypoints; ///< Features' keypoints
	cv::Mat descriptors; ///< Features' descriptors

	cv_bridge::CvImagePtr img_ptr; ///< Pointer to the frame image
	ros::Time timestamp; ///< Timestamp of the frame

// Private methods

};

} /* namespace LRM */
#endif /* FRAME_H_ */
