/*
 * mono_odometer.h
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#ifndef MONO_ODOMETER_H_
#define MONO_ODOMETER_H_

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/smart_ptr.hpp>

#include "frame.h"
#include "feature.h"
#include "motion.h"

//TEMPORARY
#include <fstream>

namespace LRM {

/*
 *
 */
class MonoOdometer {

private:

//	std::deque<Motion> queueOfMotions;	///< List of motions associated with each frame

	// Calibration data
	std::string calib_filename;
	cv::FileStorage calib_data;
	cv::Mat camera_matrix;

	// Private methods
	void configROSParam( ros::NodeHandle nh );

	//==============================================================//
	//====================Feature Extractor=========================//
	//==============================================================//
	boost::shared_ptr<FeatureHandler> f_handler;			///< Handles the operations on features

	std::deque<Frame>  queueOfFrames;	///< List of frames stored for processing

	// Topics
	std::string image_topic;			///< Name of the topic publishing the input image (ROS parameter)
	std::string feature_image_topic;	///< Name of the topic publishing the images with features (ROS parameter)

	// Types
	std::string feature_type_name;		///< Type of the features used (ROS parameter)
	feature_t featureType;				///< Type of the features used

	std::string odom_frame_id;

	int number_of_features;				///< Type of the features used
	bool show_features;					///< Display the image with the features detected
	bool show_tracks;					///< Display the image with the features tracks

	// Image publisher and subscriber
	image_transport::Publisher impub;	///< Feature image publisher
	image_transport::Subscriber imsub;	///< Source image subscriber

	// Private methods
	feature_t parseFeatureType( std::string feature_type );

	//=============================================================//
	//====================Motion Estimator=========================//
	//=============================================================//
	boost::shared_ptr<MotionEstimator> m_estimator;			///< Estimates the motion of the camera

	// Odometer publisher
	ros::Publisher odompub;

	// Transformation
	tf::Transform transformation;

	cv::Vec3f position;


	//TODO: REMOVE THIS !!!!! TEMPORARY !!!!!!!!
	std::ofstream myfile;

public:
	MonoOdometer( ros::NodeHandle nh, image_transport::ImageTransport it );
	virtual ~MonoOdometer();

	void ImageCallback( const sensor_msgs::ImageConstPtr& msg );

	const char* getImageTopic() { return image_topic.c_str(); }

};

} /* namespace LRM */
#endif /* MONO_ODOMETER_H_ */
