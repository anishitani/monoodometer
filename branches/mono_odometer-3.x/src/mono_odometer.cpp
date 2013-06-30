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

	nh.param<std::string>("CAMERA_MATRIX_TOPIC",value_str, "");
	parameter["CAMERA_MATRIX_TOPIC"] = value_str;

	nh.param<std::string>("CAMERA_MATRIX_PATH",value_str, "");
	parameter["CAMERA_MATRIX_PATH"] = value_str;

	nh.param<std::string>("INPUT_IMAGE_TOPIC", value_str, "/camera/image");
	parameter["INPUT_IMAGE_TOPIC"] = value_str;

	nh.param<std::string>("FEATURE_IMAGE_TOPIC", value_str, "image/feature");
	parameter["FEATURE_IMAGE_TOPIC"] = value_str;

	nh.param<std::string>("MATCHES_IMAGE_TOPIC", value_str, "image/matches");
	parameter["MATCHES_IMAGE_TOPIC"] = value_str;

	nh.param<std::string>("ODOMETER_REFERENCE_FRAME", value_str, "odom");
	parameter["ODOMETER_REFERENCE_FRAME"] = value_str;

	nh.param<std::string>("ROBOT_FRAME", value_str, "base_link");
	parameter["ROBOT_FRAME"] = value_str;

	ROS_DEBUG("CAMERA_MATRIX_TOPIC = %s",
			boost::any_cast<std::string>(parameter["CAMERA_MATRIX_TOPIC"]).c_str());
	ROS_DEBUG("CAMERA_MATRIX_PATH = %s",
			boost::any_cast<std::string>(parameter["CAMERA_MATRIX_PATH"]).c_str());
	ROS_DEBUG("INPUT_IMAGE_TOPIC = %s",
			boost::any_cast<std::string>(parameter["INPUT_IMAGE_TOPIC"]).c_str());
	ROS_DEBUG("FEATURE_IMAGE_TOPIC = %s",
			boost::any_cast<std::string>(parameter["FEATURE_IMAGE_TOPIC"]).c_str());
	ROS_DEBUG("MATCHES_IMAGE_TOPIC = %s",
			boost::any_cast<std::string>(parameter["MATCHES_IMAGE_TOPIC"]).c_str());
	ROS_DEBUG("ODOMETER_REFERENCE_FRAME = %s",
			boost::any_cast<std::string>(parameter["ODOMETER_REFERENCE_FRAME"]).c_str());
	ROS_DEBUG("ROBOT_FRAME = %s",
			boost::any_cast<std::string>(parameter["ROBOT_FRAME"]).c_str());

	return 0;
}

//////////////////////////////////////////////////////////////////////////
/**						MonoOdometer Class							   **/
//////////////////////////////////////////////////////////////////////////
MonoOdometer::MonoOdometer()
{
	// TODO Auto-generated destructor stub
	query_timestamp = train_timestamp = ros::Time::now();
}

/**
 *	 @brief The constructor of the Mono Odometer class. Captures and process
 * 	 a new frame and estimates the momentaneous motion.
 *
 * 	 @param[in]	nh	Node handle of the node of the monocular odometer.
 */
MonoOdometer::MonoOdometer(ros::NodeHandle &nh,
		image_transport::ImageTransport &it)
{
	ros_parameter.parse(nh);
	img_proc_parameter.parse(nh);
	mot_proc_parameter.parse(nh);


	/*
	 * Loads camera matrix.
	 */
	if(isCameraMatrixPath()){
		cv::FileStorage calib_data;
		calib_data.open(getCameraMatrixPath().c_str(), cv::FileStorage::READ);
		if(calib_data.isOpened()){
			calib_data["cameraMatrix"] >> camera_matrix;
		}
		else{
			ROS_WARN("Could not open file %s",getCameraMatrixPath().c_str());
		}
	}
	else if(isCameraMatrixTopic()){

	}
	if(camera_matrix.empty()){
		ROS_ERROR("No camera matrix was set.");
		ros::shutdown();
	}

	img_proc.setting(img_proc_parameter);
	mot_proc.setting(mot_proc_parameter);

	/*
	 * Initialize the pose and set the relation between
	 * odometer and base link.
	 */
	pose.setIdentity();
	odom_broadcaster.sendTransform(
			tf::StampedTransform(pose, ros::Time::now(), getOdometerReferenceFrame(), getRobotFrame()));

	/*
	 * Sets the image callback.
	 */
	input_image_subscriber = it.subscribe(getInputImageTopic(), 1,
			&MonoOdometer::ImageCallback, this);

	/*
	 * Sets the features image topic and matches image topic.
	 */
	output_feature_advertiser = it.advertise(getFeatureImageTopic(), 1);
	output_matches_advertiser = it.advertise(getMatchesImageTopic(), 1);

	query_timestamp = train_timestamp = ros::Time::now();
}

MonoOdometer::~MonoOdometer()
{
	// TODO Auto-generated destructor stub
}

int update_pose()
{
	return 0;
}

/**
 * @brief Converts a sensor message to a grayscale image.
 *
 * @param msg		Sensor image
 * @param image		Output image
 * @return			Error code
 */
int MonoOdometer::convertSensorMsgToImage(const sensor_msgs::ImageConstPtr &msg,
		cv_bridge::CvImageConstPtr &image)
{
	image = cv_bridge::toCvCopy(msg, "mono8");
	return 0;
}

int MonoOdometer::drawFeatureImage()
{
	feature_image.encoding = sensor_msgs::image_encodings::BGR8;
	ImageProcessor::draw_feature(query_image->image, feature_image.image,
			query_kpts);
	return 0;
}

int MonoOdometer::drawMatchesImage()
{
	matches_image.encoding = sensor_msgs::image_encodings::BGR8;
	ImageProcessor::draw_matches(query_image->image, query_kpts,
			train_image->image, train_kpts, matches, matches_image.image, mot_proc.getInlierMask());
	return 0;
}

/**
 * 	@brief The Image Callback method is responsible for handling the income
 * 	messages from the camera device and extract the frame features and
 * 	descriptors.
 *
 * 	 @param[in]	msg		Income message from the defined camera topic.
 */
void MonoOdometer::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	query_timestamp = msg->header.stamp;
	ROS_INFO("%lf",query_timestamp.toSec());
	convertSensorMsgToImage(msg, query_image);

	// Detect features and extract the features descriptors
	img_proc.detect_features(query_image->image, query_kpts);
	img_proc.extract_features(query_image->image, query_kpts, query_desc);

	if (!train_desc.empty())
	{
		img_proc.match_features(query_kpts, train_kpts, query_desc, train_desc,
				matches);

		if (matches.size() > 8)
		{
			mot_proc.matches2points(train_kpts,query_kpts,matches,train_pts,query_pts);
			cv::Mat P;
			mot_proc.estimate_motion(train_pts, query_pts, matches, camera_matrix, P);
		}

		update_pose();

		if(output_feature_advertiser.getNumSubscribers()>0){
			drawFeatureImage();
			output_feature_advertiser.publish(feature_image.toImageMsg());
		}
		if(output_matches_advertiser.getNumSubscribers()>0){
			drawMatchesImage();
			output_matches_advertiser.publish(matches_image.toImageMsg());
		}
	}

	train_kpts = query_kpts;
	train_desc = query_desc;
	train_image = query_image;
	train_timestamp = query_timestamp;

}

} /* namespace LRM */
