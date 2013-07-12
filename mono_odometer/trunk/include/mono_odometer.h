/*
 * mono_odometer.h
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#ifndef MONO_ODOMETER_H_
#define MONO_ODOMETER_H_

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <boost/smart_ptr.hpp>

#include "frame.h"
#include "feature.h"
#include "motion.h"

namespace LRM {

/*
 *
 */
class MonoOdometer {
private:
	std::deque<Frame>  queueOfFrames;	///< List of frames stored for processing
	//std::queue<Motion> queueOfMotions;	///< List of motions associated with each frame

	boost::shared_ptr<FeatureHandler> f_handler;			///< Handles the operations on features
	boost::shared_ptr<MotionEstimator> m_estimator;			///< Estimates the motion of the camera

	// Topics
	std::string image_topic;			///< Name of the topic publishing the input image (ROS parameter)
	std::string feature_image_topic;	///< Name of the topic publishing the images with features (ROS parameter)

	// Types
	std::string feature_type_name;		///< Type of the features used (ROS parameter)
	feature_t featureType;				///< Type of the features used

	int number_of_features;				///< Type of the features used
	bool show_features;					///< Display the image with the features detected
	bool show_tracks;					///< Display the image with the features tracks

	// Calibration data
	std::string calib_filename;
	cv::FileStorage calib_data;
	cv::Mat camera_matrix;

	// Image publisher and subscriber
	image_transport::Publisher pub;		///< Feature image publisher
	image_transport::Subscriber sub;	///< Source image subscriber

	// Private methods
	feature_t parseFeatureType( std::string feature_type );
	void configROSParam( ros::NodeHandle nh );

public:
	MonoOdometer( ros::NodeHandle nh, image_transport::ImageTransport it );
	virtual ~MonoOdometer();

	void ImageCallback( const sensor_msgs::ImageConstPtr& msg );

	const char* getImageTopic() { return image_topic.c_str(); }

};

} /* namespace LRM */
#endif /* MONO_ODOMETER_H_ */
