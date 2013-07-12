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
	configROSParam( nh );

	featureType = parseFeatureType(feature_type_name);

	if( featureType==NO_FEATURE ){
		ROS_ERROR("No feature or wrong type of feature defined.");
		exit(-1);
	}

	calib_data.open( calib_filename, cv::FileStorage::READ );
	calib_data[ "cameraMatrix" ] >> camera_matrix;
//	int tmp;
//	calib_data[ "image_width" ] >> tmp;

	ROS_INFO( "Source image topic: %s",image_topic.c_str() );
	ROS_INFO( "Feature image topic: %s",feature_image_topic.c_str() );
	ROS_INFO( "Feature type: %s",feature_type_name.c_str() );
	ROS_INFO( "Number of features: %d",number_of_features );
	ROS_INFO( "Show features: %s", (show_features?"yes":"no") );
	ROS_INFO( "Show tracks: %s", (show_tracks?"yes":"no") );
	if( calib_data.isOpened() ){
		ROS_INFO( "Calibration filename: %s", calib_filename.c_str() );
//		std::cout << camera_matrix << std::endl;
	}

	/*
	 *	Instatiates new feature handler and motion estimator objects
	 */
	f_handler.reset( new FeatureHandler( featureType,number_of_features ) );
	m_estimator.reset( new MotionEstimator() );

	/*
	 * Image Subscriber and Publisher
	 */
	imsub = it.subscribe( image_topic,1,&MonoOdometer::ImageCallback,this );
	impub = it.advertise( feature_image_topic, 1 );

	/*
	 * Odometer Subscriber and Publisher
	 */
	odompub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

	position = cv::Vec3f(0,0,0);

	//TEMPORARY
//	myfile.open ("/home/nishitani/Dropbox/usp/ros/mono_odometer/points.dat", std::ios::in | std::ios::ate | std::ios::app);
//	myfile << position.val[0] << ' ' << position.val[1] << ' ' << position.val[2] << std::endl;
//	myfile.close();

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

	nh.param<std::string>("calib_filename",calib_filename,"");

	nh.param<std::string>("odom_frame_id", odom_frame_id, std::string("/odom"));
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
	//TODO: The class frame should contain a frame vector (with all frames)
	//TODO: Store features and motion inside?
	queueOfFrames.push_back( Frame( msg ) );
	if( queueOfFrames.size()<2 ) {
		// TODO: Easy radius set (ROSLAUNCH)
		f_handler->setRadius( (msg->width > msg->height) ? msg->width*0.05 : msg->height*0.05 );
		queueOfFrames.back().init( f_handler );
		return;
	}
	if( queueOfFrames.size()>4 ) {
		queueOfFrames.pop_front();
	}

	int queueLast = queueOfFrames.size()-1;

	//TODO: Create a process method inside MonoOdometer
	queueOfFrames.back().process( f_handler,queueOfFrames[queueLast-1] );

	//TODO: Merge this matrix somewhere else. Maybe inside this class.
	cv::Mat P;

	m_estimator->estimate_motion(queueOfFrames[queueLast-1],queueOfFrames[queueLast],P,camera_matrix);

	cv::Matx33f R( P(cv::Range::all(),cv::Range(0,3)) );
	cv::Vec3f t( P.col(3) );
	position = R*position+t;

//	myfile.open ("/home/nishitani/Dropbox/usp/ros/mono_odometer/points.dat", std::ios::in | std::ios::ate | std::ios::app);
//	myfile << position.val[0] << ' ' << position.val[1] << ' ' << position.val[2] << std::endl;
//	myfile.close();

//	btMatrix3x3 rot_mat(  P.at<float>(0,0), P.at<float>(0,1), P.at<float>(0,2),
//	          	  	  	  P.at<float>(1,0), P.at<float>(1,1), P.at<float>(1,2),
//	          	  	  	  P.at<float>(2,0), P.at<float>(2,1), P.at<float>(2,2));
//	btVector3 t(P.at<float>(0,3), P.at<float>(1,3), P.at<float>(2,3));
//	transformation *= tf::Transform(rot_mat, t);
//
//	nav_msgs::Odometry odom_msg;
//	geometry_msgs::PoseStamped pose_msg;
//
//	pose_msg.header.stamp = queueOfFrames.back().getTimestamp();
//	pose_msg.header.frame_id = odom_frame_id;
//	tf::poseTFToMsg(tf::Transform(rot_mat, t),pose_msg.pose);
//
//	odompub.publish( pose_msg );

	if(show_features||show_tracks){
		if(show_features) queueOfFrames.back().markFeatures();
		if(show_tracks) queueOfFrames.back().markTracks( queueOfFrames[queueLast-1].getFeatList() );
		sensor_msgs::ImagePtr imgMsg = queueOfFrames.back().getImageMsg();
		impub.publish( imgMsg );
	}

	ROS_DEBUG("Number of features matched: %d", queueOfFrames.back().getNumberOfFeatures());
}

} /* namespace LRM */
