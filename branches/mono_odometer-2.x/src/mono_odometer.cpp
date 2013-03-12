/*
 * mono_odometer.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#include "mono_odometer.h"

namespace LRM {

/**
 * MonoOdometer Constructor:
 * 		The constructor of the Mono Odometer class. Captures and process
 * 	 a new frame and estimates the momentaneous motion.
 *
 * 	 @param[in]	nh	Node handle of the node of the monocular odometer.
 */
MonoOdometer::MonoOdometer( ros::NodeHandle nh, image_transport::ImageTransport it ) {

	/*
	 * ROS Information
	 */
	odomparam.configROSParam( nh );

	if( odomparam.getFeatureType()==NO_FEATURE ){
		ROS_ERROR("No feature or wrong feature type defined.");
		exit(-1);
	}

	calib_data.open( odomparam.getCalibFilename(), cv::FileStorage::READ );
	calib_data[ "cameraMatrix" ] >> camera_matrix;
	if(camera_matrix.empty()){
		ROS_ERROR("No calibration matrix was found.");
		exit(-1);
	}

	ROS_DEBUG( "Source image topic: %s"	,odomparam.getImageTopic() 			);
	ROS_DEBUG( "Feature image topic: %s",odomparam.getFeatureImageTopic() 	);
	ROS_DEBUG( "Feature type: %s"		,odomparam.getFeatureTypeName() 	);
	ROS_DEBUG( "Number of features: %d"	,odomparam.getNumberOfFeatures() 	);
	ROS_DEBUG( "Show features: %s"		, (odomparam.getShowFeatures()?"yes":"no") );
	ROS_DEBUG( "Show tracks: %s"		, (odomparam.getShowTracks()?"yes":"no") );
	if( calib_data.isOpened() ){
		ROS_DEBUG( "Calibration filename: %s", odomparam.getCalibFilename() );
	}

	/* ****************************** *
	 * Image Subscriber and Publisher *
	 **********************************/
	imsub = it.subscribe( odomparam.getImageTopic(),1,&MonoOdometer::ImageCallback,this );
	impub = it.advertise( odomparam.getFeatureImageTopic(), 1 );
}

MonoOdometer::~MonoOdometer() {
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
void MonoOdometer::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {

}

} /* namespace LRM */
