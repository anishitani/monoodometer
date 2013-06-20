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
#include "image_processor.h"
//#include "motion_processor.h"

//TEMPORARY
#include <fstream>

namespace LRM
{

class ROSParameter: public Parameter
{
public:
	ROSParameter()
	{
	}

	~ROSParameter()
	{
	}

	int parse(ros::NodeHandle nh);

};

/*
 *
 */
class MonoOdometer
{
private:
	/* *****ROS Variables******* */
	ROSParameter ros_parameter;
	image_transport::Subscriber input_image_subscriber;
	image_transport::Publisher  output_feature_advertiser;
	/* ************************* */

	/* *****Image Variables***** */
	ImageProcessor img_proc;

	ImageProcessorParameter img_proc_parameter;

	cv_bridge::CvImageConstPtr train_image; 	///< Previous image
	cv_bridge::CvImageConstPtr query_image; 	///< Current image
	cv_bridge::CvImage feature_image;

	std::vector<cv::KeyPoint> train_kpts; ///< Previous image keypoints
	std::vector<cv::KeyPoint> query_kpts; ///< Current image keypoints
	/* ************************* */

	int convertSensorMsgToImage(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImageConstPtr &image);
	int drawFeatureImage();

public:
	MonoOdometer();
	MonoOdometer(ros::NodeHandle &nh, image_transport::ImageTransport &it);
	~MonoOdometer();

	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

	std::string getInputImageTopic()
	{
		return ros_parameter.getParameterByName<std::string>(
				"INPUT_IMAGE_TOPIC");
	}

	std::string getFeatureImageTopic()
	{
		return ros_parameter.getParameterByName<std::string>(
				"FEATURE_IMAGE_TOPIC");
	}
};

} /* namespace LRM */
#endif /* MONO_ODOMETER_H_ */
