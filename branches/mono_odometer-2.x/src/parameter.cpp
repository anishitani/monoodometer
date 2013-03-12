/*
 * parameter.cpp
 *
 *  Created on: Mar 12, 2013
 *      Author: nishitani
 */

#include "parameter.h"

namespace LRM{

/**
 * Method configROSParam:
 * 		Get the parameters defined by the ROS launch file.
 *
 * 	 @param[in]	nh	Handler of the ROS node
 */
void OdomParam::configROSParam( ros::NodeHandle nh ) {
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
 * Method parseFeatureType:
 * 		Parse the features types.
 *
 * 	 @param[in]	feature_type	String defining the type of feature used with the detector
 * 	 @return	Returns the type of feature
 */
void OdomParam::parseFeatureType( std::string feature_type ) {
	if(strcmp( feature_type.c_str(),"SHI_TOMASI")==0 ){
		this->feature_type = SHI_TOMASI;
	}
	if(strcmp( feature_type.c_str(),"HARRIS")==0 ){
		this->feature_type = HARRIS;
	}
	if(strcmp( feature_type.c_str(),"ORB")==0 ){
		this->feature_type = ORB;
	}
	if(strcmp( feature_type.c_str(),"FAST")==0 ){
		this->feature_type = FAST;
	}
	if(strcmp( feature_type.c_str(),"SURF")==0 ){
		this->feature_type = SURF;
	}
	if(strcmp( feature_type.c_str(),"SIFT")==0 ){
		this->feature_type = SIFT;
	}

	this->feature_type = NO_FEATURE;
}

}




