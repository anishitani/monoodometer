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

class Parameter
{

public:
	Parameter()
	{
	}

	virtual ~Parameter()
	{
	}

//	void configROSParam( ros::NodeHandle nh );
//	void parseFeatureType();
	/*
	 //gets
	 const char* getImageTopic()			{ return image_topic.c_str(); }
	 const char* getFeatureImageTopic()	{ return feature_image_topic.c_str(); }
	 const char* getFeatureTypeName()	{ return feature_type_name.c_str(); }
	 const char* getCalibFilename()		{ return calib_filename.c_str(); }
	 feature_t getFeatureType()	{ return feature_type; }
	 int getNumberOfFeatures()	{ return number_of_features; }
	 uint getBundleSize()		{ return bundle_window_size; }
	 bool getDrawKeypoints()		{ return draw_keypoints; }
	 bool getDrawTracks()		{ return draw_tracks; }
	 bool getDrawPair()			{ return draw_pair; }

	 const char* getOdomTopic()	{ return odom_topic.c_str(); }

	 private:
	 // Calibration file
	 std::string calib_filename;

	 // Topics
	 std::string image_topic;			///< Name of the topic publishing the input image (ROS parameter)
	 std::string feature_image_topic;	///< Name of the topic publishing the images with features (ROS parameter)
	 std::string odom_topic;

	 // Types
	 std::string feature_type_name;		///< Type of the features used (ROS parameter)
	 feature_t feature_type;				///< Type of the features used

	 std::string odom_frame_id;

	 int number_of_features;				///< Max number of features detected
	 int bundle_window_size;				///< Size of the Bundle Adjustment Window (default 2)
	 bool draw_keypoints;				///< Display the image with the features detected
	 bool draw_tracks;					///< Display the image with the features tracks
	 bool draw_pair;						///< Display the last and current images linking the matches
	 */

protected:
	std::map<std::string, boost::any> parameter;

public:
	virtual int parse(ros::NodeHandle) = 0;

	template<class T>
	T getParameterByName(std::string name)
	{
		return boost::any_cast < T > (parameter[name]);
	}

	int setParameterByName(std::string name, boost::any value)
	{
		parameter[name] = value;
		return 0;
	}

};

}

#endif /* ODOM_PARAMETER_H_ */
