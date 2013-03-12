/*
 * parameter.h
 *
 *  Created on: Mar 12, 2013
 *      Author: nishitani
 */

#ifndef ODOM_PARAMETER_H_
#define ODOM_PARAMETER_H_

#include "core.h"

namespace LRM
{

class OdomParam
{
public:
	OdomParam(){};
	~OdomParam(){};

	void configROSParam( ros::NodeHandle nh );
	void parseFeatureType( std::string feature_type );

	//gets
	const char* getImageTopic()			{ return image_topic.c_str(); }
	const char* getFeatureImageTopic()	{ return feature_image_topic.c_str(); }
	const char* getFeatureTypeName()	{ return feature_type_name.c_str(); }
	const char* getCalibFilename()		{ return calib_filename.c_str(); }
	feature_t getFeatureType()	{ return feature_type; }
	int getNumberOfFeatures()	{ return number_of_features; }
	bool getShowFeatures()		{ return show_features; }
	bool getShowTracks()		{ return show_tracks; }

private:
	// Calibration file
	std::string calib_filename;

	// Topics
	std::string image_topic;			///< Name of the topic publishing the input image (ROS parameter)
	std::string feature_image_topic;	///< Name of the topic publishing the images with features (ROS parameter)

	// Types
	std::string feature_type_name;		///< Type of the features used (ROS parameter)
	feature_t feature_type;				///< Type of the features used

	std::string odom_frame_id;

	int number_of_features;				///< Type of the features used
	bool show_features;					///< Display the image with the features detected
	bool show_tracks;					///< Display the image with the features tracks
};

}


#endif /* ODOM_PARAMETER_H_ */
