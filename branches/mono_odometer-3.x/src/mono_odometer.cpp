/*
 * mono_odometer.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#include "mono_odometer.h"

namespace LRM
{

/*
 * INPUT_IMAGE_TOPIC
 */

//////////////////////////////////////////////////////////////////////////
/**						ROS Parameter Class							   **/
//////////////////////////////////////////////////////////////////////////
int ROSParameter::parse(ros::NodeHandle nh)
{
	std::string value_str;

	nh.param<std::string>("INPUT_IMAGE_TOPIC", value_str, "/camera/image");
	parameter["INPUT_IMAGE_TOPIC"] = value_str;

	nh.param<std::string>("FEATURE_IMAGE_TOPIC", value_str, "image/feature");
		parameter["FEATURE_IMAGE_TOPIC"] = value_str;

	ROS_DEBUG(
			"INPUT_IMAGE_TOPIC = %s", boost::any_cast<std::string>(parameter["INPUT_IMAGE_TOPIC"]).c_str());
	ROS_DEBUG(
			"FEATURE_IMAGE_TOPIC = %s", boost::any_cast<std::string>(parameter["FEATURE_IMAGE_TOPIC"]).c_str());

	return 0;
}

//////////////////////////////////////////////////////////////////////////
/**						MonoOdometer Class							   **/
//////////////////////////////////////////////////////////////////////////
MonoOdometer::MonoOdometer()
{
	// TODO Auto-generated destructor stub
}

/**
 * MonoOdometer Constructor:
 * 		The constructor of the Mono Odometer class. Captures and process
 * 	 a new frame and estimates the momentaneous motion.
 *
 * 	 @param[in]	nh	Node handle of the node of the monocular odometer.
 */
MonoOdometer::MonoOdometer(ros::NodeHandle &nh,
		image_transport::ImageTransport &it)
{
	ros_parameter.parse(nh);
	img_proc_parameter.parse(nh);

	img_proc.setting(img_proc_parameter);

	//Setting CALLBACK
	input_image_subscriber = it.subscribe(getInputImageTopic(), 1,
			&MonoOdometer::ImageCallback, this);

	output_feature_advertiser = it.advertise(getFeatureImageTopic(),1);
}

MonoOdometer::~MonoOdometer()
{
	// TODO Auto-generated destructor stub
}

/**
 * Method convertSensorMsgToImage:
 * 		Converts a sensor message to a grayscale image.
 *
 * @param msg		Sensor image
 * @param image		Output image
 * @return			Error code
 */
int MonoOdometer::convertSensorMsgToImage(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImageConstPtr &image)
{
	image = cv_bridge::toCvCopy(msg,"mono8");
	return 0;
}

int MonoOdometer::drawFeatureImage()
{
	feature_image.encoding = sensor_msgs::image_encodings::BGR8;
	ImageProcessor::draw_feature(query_image->image, feature_image.image, query_kpts);
	return 0;
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
	convertSensorMsgToImage(msg,query_image);

	img_proc.detect_features(query_image->image, query_kpts);

	//feature vector?
//	if(first_run){
//		first_run = false;
//		return;
//	}
//	else{
//		ImageProcessor::match_feature(matches, query_feature, train_feature, match_param);
//		MotionProcessor::estimate_motion(train_feature, query_feature, matches, rotation, translation);
//		update_pose(rotation, translation);
//	}

	drawFeatureImage();

	output_feature_advertiser.publish(feature_image.toImageMsg());

}

} /* namespace LRM */
