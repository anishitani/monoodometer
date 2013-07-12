/*
 * mono_odometer.h
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#ifndef MONO_ODOMETER_H_
#define MONO_ODOMETER_H_

#include "core.h"
#include "parameter.h"
#include "frame.h"
#include "feature.h"
#include "motion.h"

//TEMPORARY
#include <fstream>

namespace LRM
{

/*
 *
 */
class MonoOdometer
{

private:

	//	std::deque<Motion> queueOfMotions;	///< List of motions associated with each frame

	// Calibration data
	cv::FileStorage calib_data;
	cv::Mat camera_matrix;

	// Odometer parameters
	OdomParam odomparam;


	cv_bridge::CvImage outImg;

	//=============================================================//
	//====================Feature Handler =========================//
	//=============================================================//
	boost::shared_ptr<FeatureHandler> f_handler; ///< Handles the operations on features

	std::deque<Frame> queueOfFrames; ///< List of frames stored for processing
	std::deque<std::vector<cv::DMatch> > queueOfMatches; ///< List of frames stored for processing

	// Image publisher and subscriber
	image_transport::Publisher impub; ///< Feature image publisher
	image_transport::Subscriber imsub; ///< Source image subscriber

	// Private methods
	feature_t parseFeatureType(std::string feature_type);

	//=============================================================//
	//====================Motion Estimator=========================//
	//=============================================================//
	boost::shared_ptr<MotionEstimator> m_estimator; ///< Estimates the motion of the camera

	cv::Mat P;

	// Odometer publisher
	ros::Publisher odompub;

	// Transformation
	tf::Transform transformation;

	cv::Vec3f position;

public:
	MonoOdometer(ros::NodeHandle nh, image_transport::ImageTransport it);
	virtual ~MonoOdometer();

	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

	const char* getImageTopic()
	{
		return odomparam.getImageTopic();
	}

};

} /* namespace LRM */
#endif /* MONO_ODOMETER_H_ */
