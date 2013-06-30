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
#include "motion_processor.h"

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

class MonoOdometer
{
private:
	/* *****MonoOdometer Variables******* */
	ros::Time query_timestamp;
	ros::Time train_timestamp;

	cv::Mat camera_matrix;
	/* ********************************** */

	/* *****ROS Variables******* */
	ROSParameter ros_parameter;
	image_transport::Subscriber input_image_subscriber;
	image_transport::Publisher output_feature_advertiser;
	image_transport::Publisher output_matches_advertiser;
	/* ************************* */

	/* *****Image Variables***** */
	ImageProcessor img_proc;

	ImageProcessorParameter img_proc_parameter;

	cv_bridge::CvImageConstPtr train_image; ///< Previous image
	cv_bridge::CvImageConstPtr query_image; ///< Current image
	cv_bridge::CvImage feature_image;
	cv_bridge::CvImage matches_image;

	std::vector<cv::KeyPoint> train_kpts; ///< Previous image keypoints
	std::vector<cv::KeyPoint> query_kpts; ///< Current image keypoints

	std::vector<cv::Point2d> train_pts;
	std::vector<cv::Point2d> query_pts;

	cv::Mat train_desc;
	cv::Mat query_desc;

	std::vector<cv::DMatch> matches;
	/* ************************* */

	/* *****Motion Variables***** */
	MotionProcessor mot_proc;

	MotionProcessorParameter mot_proc_parameter;

	tf::Transform pose;
	tf::TransformBroadcaster odom_broadcaster;
	/* ************************** */

	int update_pose();
	int convertSensorMsgToImage(const sensor_msgs::ImageConstPtr &msg,
			cv_bridge::CvImageConstPtr &image);
	int drawFeatureImage();
	int drawMatchesImage();

public:
	MonoOdometer();
	MonoOdometer(ros::NodeHandle &nh, image_transport::ImageTransport &it);
	~MonoOdometer();

	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

//	void matches2points(const std::vector<cv::KeyPoint>& query,
//			const std::vector<cv::KeyPoint>& train,
//			const std::vector<cv::DMatch>& matches,
//			std::vector<cv::Point2d> &query_pts,
//			std::vector<cv::Point2d> &train_pts);

	bool isCameraMatrixPath()
	{
		std::string camera_matrix_path = ros_parameter.getParameterByName<std::string>("CAMERA_MATRIX_PATH");
		return camera_matrix_path.compare("")==0 ? false : true;

	}

	bool isCameraMatrixTopic()
	{
		return false;
	}

	std::string getCameraMatrixPath()
	{
		return ros_parameter.getParameterByName<std::string>(
				"CAMERA_MATRIX_PATH");
	}

	std::string getCameraMatrixTopic()
	{
		return ros_parameter.getParameterByName<std::string>(
				"CAMERA_MATRIX_TOPIC");
	}

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

	std::string getMatchesImageTopic()
	{
		return ros_parameter.getParameterByName<std::string>(
				"MATCHES_IMAGE_TOPIC");
	}

	std::string getOdometerReferenceFrame()
	{
		return ros_parameter.getParameterByName<std::string>(
				"ODOMETER_REFERENCE_FRAME");
	}

	std::string getRobotFrame()
	{
		return ros_parameter.getParameterByName<std::string>(
				"ROBOT_FRAME");
	}
};

} /* namespace LRM */
#endif /* MONO_ODOMETER_H_ */
