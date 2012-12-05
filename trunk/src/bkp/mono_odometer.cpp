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
 * 		The constructor of the Mono Odometer class.
 * 		The program parameters are defined from ROS paramaters or using default
 * 	parameters.
 *
 * 	 @param[in]	nh	Node handle of the node of the monocular odometer.
 * 	 @param[in]	it	Image transport handles the subscribe and advertise of images.
 */
MonoOdometer::MonoOdometer( ros::NodeHandle nh, image_transport::ImageTransport it ) {

	/*
	 * ROS Information
	 */
	configROSParam( nh );

	featureType = parseFeatureType(feature_type_name);

	if( featureType==NO_FEATURE ){
		ROS_ERROR("No feature or wrong type of feature defined.");
		exit(-1);
	}

	ROS_INFO( "Source image topic: %s",image_topic.c_str() );
	ROS_INFO( "Feature image topic: %s",feature_image_topic.c_str() );
	ROS_INFO( "Feature type: %s",feature_type_name.c_str() );
	ROS_INFO( "Number of features: %d",number_of_features );
	ROS_INFO( "Show features: %s", (show_features?"yes":"no") );
	ROS_INFO( "Show tracks: %s", (show_tracks?"yes":"no") );

	/*
	 *	Instatiates new feature handler and motion estimator objects
	 */
	f_handler.reset( new FeatureHandler( featureType,number_of_features ) );

	/*
	 * Subscriber and Publisher
	 */
	sub = it.subscribe( image_topic,1,&MonoOdometer::ImageCallback,this );
	pub = it.advertise( feature_image_topic, 1 );
}

MonoOdometer::~MonoOdometer() {
	// TODO Auto-generated destructor stub
}

/**
 * Method parseFeatureType:
 * 		Parse the features types.
 *
 * 	 @param[in]	feature_type	String defining the type of feature used with the detector
 * 	 @return	Returns the type of feature
 */
feature_t MonoOdometer::parseFeatureType( std::string feature_type ) {
	if(strcmp( feature_type.c_str(),"SHI_TOMASI")==0 ){
		return SHI_TOMASI;
	}
	if(strcmp( feature_type.c_str(),"HARRIS")==0 ){
		return HARRIS;
	}
	if(strcmp( feature_type.c_str(),"ORB")==0 ){
		return ORB;
	}
	if(strcmp( feature_type.c_str(),"FAST")==0 ){
		return FAST;
	}
	if(strcmp( feature_type.c_str(),"SURF")==0 ){
		return SURF;
	}
	if(strcmp( feature_type.c_str(),"SIFT")==0 ){
		return SIFT;
	}

	return NO_FEATURE;
}

/**
 * Method configROSParam:
 * 		Get the parameters defined in by ROS launch file.
 *
 * 	 @param[in]	nh	Handler of the ROS node
 */
void MonoOdometer::configROSParam( ros::NodeHandle nh ) {
	nh.param<std::string>("image_topic",image_topic,"/camera/image");
	nh.param<std::string>("feature_image_topic",feature_image_topic,"/image/feature_image");

	nh.param<std::string>("feature_type",feature_type_name,"SHI_TOMASI");
	nh.param<int>("number_of_features",number_of_features,50);
	nh.param<bool>("show_features",show_features,false);
	nh.param<bool>("show_tracks",show_tracks,false);
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
	// TODO: Read the type of features used as a node parameter

	queueOfFrames.push_back( Frame( msg ) );
	/*
	 * If only one frame is available on the queue, the features and descriptors
	 * are initialized and then proceed wait for the next frame.
	 */
	if( queueOfFrames.size()<2 ) {
		double radius = (msg->height > msg->width) ? msg->height*0.05 : msg->width*0.05;
		f_handler->setRadius( radius );
		queueOfFrames.back().init( f_handler );
		return;
	}
	// The queue supports a limited number of frames.
	if( queueOfFrames.size()>4 ) {
		queueOfFrames.pop_front();
	}

	int queuePos = queueOfFrames.size()-1;

	/*
	 * Process the frame acquiring features, descriptors and matching them with the
	 * last frame processed.
	 */
	queueOfFrames.back().process( f_handler,queueOfFrames[queuePos-1] );

	// Display a image with the elements processed.
	if(show_features||show_tracks){
		if(show_features) queueOfFrames.back().markFeatures();
		if(show_tracks) queueOfFrames.back().markTracks( queueOfFrames[queuePos-1].getFeatList() );
		sensor_msgs::ImagePtr imgMsg = queueOfFrames.back().getImageMsg();
		pub.publish( imgMsg );
	}
//	ROS_DEBUG("Number of features matched: %d", queueOfFrames.back().getNumberOfFeatures());

//	queueOfFrames.pop_front();
}

} /* namespace LRM */
