/*
 * mono_odometer.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#include "mono_odometer.h"

namespace LRM
{

/**
 * MonoOdometer Constructor:
 * 		The constructor of the Mono Odometer class. Captures and process
 * 	 a new frame and estimates the momentaneous motion.
 *
 * 	 @param[in]	nh	Node handle of the node of the monocular odometer.
 */
MonoOdometer::MonoOdometer(ros::NodeHandle nh,
		image_transport::ImageTransport it)
{

	/*
	 * ROS Information
	 */
	odomparam.configROSParam(nh);
	odomparam.parseFeatureType();

	if (odomparam.getFeatureType() == NO_FEATURE)
	{
		ROS_ERROR("No feature or wrong feature type defined.");
		exit(-1);
	}

	calib_data.open(odomparam.getCalibFilename(), cv::FileStorage::READ);
	if (!calib_data.isOpened())
	{
		ROS_ERROR("No calibration matrix was found.");
		exit(-1);
	}
	calib_data["cameraMatrix"] >> camera_matrix;

	ROS_DEBUG( "Source image topic: %s", odomparam.getImageTopic());
	ROS_DEBUG( "Feature image topic: %s", odomparam.getFeatureImageTopic());
	ROS_DEBUG( "Feature type: %s", odomparam.getFeatureTypeName());
	ROS_DEBUG( "Number of features: %d", odomparam.getNumberOfFeatures());
	ROS_DEBUG( "Draw keypoints: %s", (odomparam.getDrawKeypoints()?"yes":"no"));
	ROS_DEBUG( "Draw tracks: %s", (odomparam.getDrawTracks()?"yes":"no"));
	ROS_DEBUG( "Calibration filename: %s", odomparam.getCalibFilename());

	/* ************************** *
	 * Feature and Motion Handler *
	 * ************************** */
	f_handler.reset(
			new FeatureHandler(odomparam.getFeatureType(),
					odomparam.getNumberOfFeatures()));
	m_estimator.reset(new MotionEstimator());

	/* ****************************** *
	 * Image Subscriber and Publisher *
	 * ****************************** */
	imsub = it.subscribe(odomparam.getImageTopic(), 1,
			&MonoOdometer::ImageCallback, this);
	impub = it.advertise(odomparam.getFeatureImageTopic(), 1);

	outImg.encoding = sensor_msgs::image_encodings::BGR8;
}

MonoOdometer::~MonoOdometer()
{
	// TODO Auto-generated destructor stub
}

/**
 * Method ImageCallback:
 * 		The Image Callback method is responsible for handling the income
 * 	messages from the camera device and extract the frame features and
 * 	descriptors.
 *
 * 	 @param[in]	msg		Income message from the defined camera topic.
 */
void MonoOdometer::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// New frame from camera
	Frame frame(msg);

	frame.process(*f_handler);

	if (queueOfFrames.empty())
	{
		queueOfFrames.push_back(frame);
		return;
	}
	else
	{
		std::vector<cv::DMatch> matches;

		if (queueOfFrames.size() >= odomparam.getBundleSize())
		{
			queueOfFrames.pop_front();
			queueOfMatches.pop_front();
		}

		Frame::match(*f_handler, queueOfFrames.back(), frame, matches);

		Frame::motion(*m_estimator, queueOfFrames.back(), frame, matches,
				camera_matrix, P);

		Frame::draw(outImg.image, queueOfFrames.back(), frame, matches,
				odomparam, m_estimator->getInliers());

		queueOfFrames.push_back(frame);
		queueOfMatches.push_back(matches);
	}

	impub.publish(outImg.toImageMsg());
}

} /* namespace LRM */
