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
/**
 *  List of parameter regarding ROS configuration for the launch file.
 *    - @b INTRINSIC_MATRIX_PATH: Path to the intrinsic matrix K file. (@b File @b format:YAML - @b Default: empty)
 *    - @b INPUT_IMAGE_TOPIC: Topic name of the input image. (@b Default: /camera/image)
 *    - @b FEATURE_IMAGE_TOPIC: Name of the topic where the output image with the features detect will be published. (@b Default: images/feature)
 *    - @b MATCHES_IMAGE_TOPIC: Name of the topic where the output image with the features matched will be published. (@b Default: images/matches)
 *    - @b OPTFLOW_IMAGE_TOPIC: Name of the topic where the output image with the feature displacement will be published. (@b Default: images/optflow)
 *    - @b ODOMETRY_TOPIC: Name of the topic where the odometry will be published. (@b Default: odom)
 *    - @b ODOMETER_REFERENCE_FRAME: Fixed frame used as reference for the odometer. (@b Default: odom)
 *    - @b ROBOT_FRAME: The frame which all sensors are referenced to. Usually the base link. (@b Default: base_link)
 *    - @b SENSOR_FRAME: The image capturing sensor frame. (@b Default: camera)
 */

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

	cv::Mat K;
	/* ********************************** */

	/* *****ROS Variables******* */
	ROSParameter ros_parameter;
	image_transport::Subscriber input_image_subscriber;
	image_transport::CameraSubscriber input_camera_subscriber;
	image_transport::Publisher output_feature_advertiser;
	image_transport::Publisher output_matches_advertiser;
	image_transport::Publisher output_optflow_advertiser;
	image_transport::Publisher output_displacement_advertiser;
	ros::Publisher odometry_advertiser;
	/* ************************* */

	/**
	 * @todo The Image Processor object will store the points,
	 * descriptors, matches and any other data pertinent to the image
	 * processing step.
	 */

	/* *****Image Variables***** */
	ImageProcessor img_proc;

	ImageProcessorParameter img_proc_parameter;

	cv_bridge::CvImageConstPtr train_image; ///< Previous image
	cv_bridge::CvImageConstPtr query_image; ///< Current image
	cv_bridge::CvImage feature_image;
	cv_bridge::CvImage matches_image;
	cv_bridge::CvImage optflow_image;
	cv_bridge::CvImage displacement_image;

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
	tf::StampedTransform base_to_sensor;
	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformListener tf_listener;

	nav_msgs::Odometry odometry;
	/* ************************** */

	bool find_rich_match();
	bool find_optflow_match();
	int update_pose();
	int convertSensorMsgToImage(const sensor_msgs::ImageConstPtr &msg,
			cv_bridge::CvImageConstPtr &image);
	int drawFeatureImage();
	int drawMatchesImage();
	int drawOptFlowImage();
	int drawDisplacementImage();

public:
	MonoOdometer();
	MonoOdometer(ros::NodeHandle &nh, image_transport::ImageTransport &it);
	~MonoOdometer();

	void CallbackHandler(const sensor_msgs::ImageConstPtr& img,
			const sensor_msgs::CameraInfoConstPtr& cam);
	void ImageCallback(const sensor_msgs::ImageConstPtr& img);
	void CameraCallback(const sensor_msgs::ImageConstPtr& img,
			const sensor_msgs::CameraInfoConstPtr& cam);

//	void matches2points(const std::vector<cv::KeyPoint>& query,
//			const std::vector<cv::KeyPoint>& train,
//			const std::vector<cv::DMatch>& matches,
//			std::vector<cv::Point2d> &query_pts,
//			std::vector<cv::Point2d> &train_pts);

	bool isIntrinsicMatrixPath()
	{
		std::string intrinsic_matrix_path = ros_parameter.getParameterByName<
				std::string>("INTRINSIC_MATRIX_PATH");
		return intrinsic_matrix_path.compare("") == 0 ? false : true;

	}

	std::string getIntrinsicMatrixPath()
	{
		return ros_parameter.getParameterByName<std::string>(
				"INTRINSIC_MATRIX_PATH");
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

	std::string getOptFlowImageTopic()
	{
		return ros_parameter.getParameterByName<std::string>(
				"OPTFLOW_IMAGE_TOPIC");
	}

	std::string getDisplacementImageTopic()
	{
		return ros_parameter.getParameterByName<std::string>(
				"DISPLACEMENT_IMAGE_TOPIC");
	}

	std::string getOdometryTopic()
	{
		return ros_parameter.getParameterByName<std::string>("ODOMETRY_TOPIC");
	}

	std::string getOdometerReferenceFrame()
	{
		return ros_parameter.getParameterByName<std::string>(
				"ODOMETER_REFERENCE_FRAME");
	}

	std::string getRobotFrame()
	{
		return ros_parameter.getParameterByName<std::string>("ROBOT_FRAME");
	}

	std::string getSensorFrame()
	{
		return ros_parameter.getParameterByName<std::string>("SENSOR_FRAME");
	}
};

} /* namespace LRM */
#endif /* MONO_ODOMETER_H_ */
