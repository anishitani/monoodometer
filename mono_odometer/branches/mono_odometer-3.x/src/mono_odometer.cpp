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

	nh.param<std::string>("INTRINSIC_MATRIX_PATH", value_str, "");
	parameter["INTRINSIC_MATRIX_PATH"] = value_str;

	nh.param<std::string>("INPUT_IMAGE_TOPIC", value_str, "/camera/image");
	parameter["INPUT_IMAGE_TOPIC"] = value_str;

	nh.param<std::string>("FEATURE_IMAGE_TOPIC", value_str, "images/feature");
	parameter["FEATURE_IMAGE_TOPIC"] = value_str;

	nh.param<std::string>("OPTFLOW_IMAGE_TOPIC", value_str, "images/optflow");
	parameter["OPTFLOW_IMAGE_TOPIC"] = value_str;

	nh.param<std::string>("MATCHES_IMAGE_TOPIC", value_str, "images/matches");
	parameter["MATCHES_IMAGE_TOPIC"] = value_str;

	nh.param<std::string>("DISPLACEMENT_IMAGE_TOPIC", value_str,
			"images/displacement");
	parameter["DISPLACEMENT_IMAGE_TOPIC"] = value_str;

	nh.param<std::string>("ODOMETRY_TOPIC", value_str, "odom");
	parameter["ODOMETRY_TOPIC"] = value_str;

	nh.param<std::string>("POSE_TOPIC", value_str, "pose");
	parameter["POSE_TOPIC"] = value_str;

	nh.param<std::string>("ODOMETER_REFERENCE_FRAME", value_str, "odom");
	parameter["ODOMETER_REFERENCE_FRAME"] = value_str;

	nh.param<std::string>("ROBOT_FRAME", value_str, "base_link");
	parameter["ROBOT_FRAME"] = value_str;

	nh.param<std::string>("SENSOR_FRAME", value_str, "camera");
	parameter["SENSOR_FRAME"] = value_str;

	ROS_DEBUG("INTRINSIC_MATRIX_TOPIC = %s",
			boost::any_cast<std::string>(parameter["INTRINSIC_MATRIX_TOPIC"]).c_str());
	ROS_DEBUG("INTRINSIC_MATRIX_PATH = %s",
			boost::any_cast<std::string>(parameter["INTRINSIC_MATRIX_PATH"]).c_str());
	ROS_DEBUG("INPUT_IMAGE_TOPIC = %s",
			boost::any_cast<std::string>(parameter["INPUT_IMAGE_TOPIC"]).c_str());
	ROS_DEBUG("FEATURE_IMAGE_TOPIC = %s",
			boost::any_cast<std::string>(parameter["FEATURE_IMAGE_TOPIC"]).c_str());
	ROS_DEBUG("MATCHES_IMAGE_TOPIC = %s",
			boost::any_cast<std::string>(parameter["MATCHES_IMAGE_TOPIC"]).c_str());
	ROS_DEBUG("OPTFLOW_IMAGE_TOPIC = %s",
			boost::any_cast<std::string>(parameter["OPTFLOW_IMAGE_TOPIC"]).c_str());
	ROS_DEBUG("ODOMETRY_TOPIC = %s",
			boost::any_cast<std::string>(parameter["ODOMETRY_TOPIC"]).c_str());
	ROS_DEBUG("POSE_TOPIC = %s",
			boost::any_cast<std::string>(parameter["POSE_TOPIC"]).c_str());
	ROS_DEBUG("ODOMETER_REFERENCE_FRAME = %s",
			boost::any_cast<std::string>(parameter["ODOMETER_REFERENCE_FRAME"]).c_str());
	ROS_DEBUG("ROBOT_FRAME = %s",
			boost::any_cast<std::string>(parameter["ROBOT_FRAME"]).c_str());
	ROS_DEBUG("SENSOR_FRAME = %s",
			boost::any_cast<std::string>(parameter["SENSOR_FRAME"]).c_str());

	return 0;
}

//////////////////////////////////////////////////////////////////////////
/**						MonoOdometer Class							   **/
//////////////////////////////////////////////////////////////////////////
MonoOdometer::MonoOdometer()
{
	// TODO Auto-generated destructor stub
	query_timestamp = train_timestamp = ros::Time::now();

	matcher = new Matcher(matcher_parameter);
}

/**
 *	 @brief The constructor of the Mono Odometer class. Captures and process
 * 	 a new frame and estimates the momentaneous motion.
 *
 * 	 @param[in]	nh	Node handle of the node of the monocular odometer.
 * 	 @param[in] it 	Image transport handler.
 */
MonoOdometer::MonoOdometer(ros::NodeHandle &nh,
		image_transport::ImageTransport &it)
{
	ros_parameter.parse(nh);
	img_proc_parameter.parse(nh);
	mot_proc_parameter.parse(nh);

	matcher = new Matcher(matcher_parameter);

	/*
	 * Loads intrinsic parameters matrix.
	 */
	if (isIntrinsicMatrixPath())
	{
		/*
		 * Sets the image callback.
		 */
		input_image_subscriber = it.subscribe(getInputImageTopic(), 1,
				&MonoOdometer::ImageCallback, this);

	}
	else
	{
		input_camera_subscriber = it.subscribeCamera(getInputImageTopic(), 1,
				&MonoOdometer::CameraCallback, this);
	}

	img_proc.setting(img_proc_parameter);
	mot_proc.setting(mot_proc_parameter);

	/*
	 * Initialize the pose and set the relation between
	 * odometer and base link.
	 */
	pose.setIdentity();
	odom_broadcaster.sendTransform(
			tf::StampedTransform(pose, ros::Time::now(),
					getOdometerReferenceFrame(), getRobotFrame()));

	/*
	 * Sets the features image topic and matches image topic.
	 */
	output_feature_advertiser = it.advertise(getFeatureImageTopic(), 1);
	output_matches_advertiser = it.advertise(getMatchesImageTopic(), 1);
	output_optflow_advertiser = it.advertise(getOptFlowImageTopic(), 1);
	output_displacement_advertiser = it.advertise(getDisplacementImageTopic(),
			1);

	odometry_advertiser = nh.advertise<nav_msgs::Odometry>(getOdometryTopic(),
			1);
	pose_advertiser = nh.advertise<geometry_msgs::PoseStamped>(getPoseTopic(),
			1);

	query_timestamp = train_timestamp = ros::Time::now();

	ROS_INFO("MonoOdometer initialized!");
}

MonoOdometer::~MonoOdometer()
{
	// TODO Auto-generated destructor stub
}

bool MonoOdometer::find_rich_match()
{
	/**
	 * @todo Incorporate de descriptors extraction on the match process.
	 */
	// Detect features and extract the features descriptors
	img_proc.detect_features(query_image->image, query_kpts);
	img_proc.extract_features(query_image->image, query_kpts, query_desc);
	if (train_desc.empty())
		// Returns on first run
		return false;
	else
	{ // Find Matches if it's not first run
		img_proc.match_features(query_desc, train_desc, matches);
		mot_proc.matches2points(train_kpts, query_kpts, matches, train_pts,
				query_pts);

		return true;
	}
}

bool MonoOdometer::find_optflow_match()
{
	img_proc.detect_features(query_image->image, query_kpts);
	if (train_kpts.empty())
		return false;
	else
	{
		img_proc.match_features_optflow(query_image->image, train_image->image,
				query_kpts, train_kpts, matches);
		mot_proc.matches2points(train_kpts, query_kpts, matches, train_pts,
				query_pts);
		return true;
	}
}

bool MonoOdometer::find_optflow_libviso()
{
	int r=query_image->image.rows,c = query_image->image.cols;
	int dims[3] = {c,r,c};
	if (train_kpts.empty()){
		ROS_INFO("First RUN!");
		query_kpts.push_back(cv::KeyPoint(1,1,1));
		matcher->pushBack(query_image->image.data,dims,false);
		return false;
	}
	else
	{
		train_kpts.clear();
		query_kpts.clear();
		matches.clear();
		matcher->pushBack(query_image->image.data,dims,false);
		matcher->matchFeatures(0);
		matcher->bucketFeatures(2,50,50);
		cv::vector<Matcher::p_match> p_matches = matcher->getMatches();
		for(size_t i=0 ; i<p_matches.size() ; i++){
			Matcher::p_match m = p_matches[i];
			train_kpts.push_back(cv::KeyPoint(m.u1p,m.v1p,1));
			query_kpts.push_back(cv::KeyPoint(m.u1c,m.v1c,1));
			matches.push_back(cv::DMatch(i,i,1));
		}
		mot_proc.matches2points(train_kpts, query_kpts, matches, train_pts,
				query_pts);
		return true;
	}
}

int MonoOdometer::update_pose()
{
	cv::Mat P = mot_proc.getCameraMatrix();

//	P = P.inv();

	tf::Matrix3x3 R(P.at<double>(0, 0), P.at<double>(0, 1), P.at<double>(0, 2),
			P.at<double>(1, 0), P.at<double>(1, 1), P.at<double>(1, 2),
			P.at<double>(2, 0), P.at<double>(2, 1), P.at<double>(2, 2));

	tf::Vector3 T(P.at<double>(0, 3), P.at<double>(1, 3), P.at<double>(2, 3));
	/// @todo Remove the factor over the translation.
	tf::Transform motion(R, 0.5*T);

	std::string error_msg;
	if (tf_listener.canTransform(getRobotFrame(), getSensorFrame(),
			ros::Time::now(), &error_msg))
	{
		tf_listener.lookupTransform(getRobotFrame(), getSensorFrame(),
				ros::Time::now(), base_to_sensor);
	}
	else
	{
		ROS_WARN_THROTTLE(10.0,
				"The tf from '%s' to '%s' does not seem to be available, "
						"will assume it as identity!", getRobotFrame().c_str(),
				getSensorFrame().c_str());
		ROS_DEBUG("Transform error: %s", error_msg.c_str());
		base_to_sensor.setIdentity();
	}

	pose *= motion;

	tf::Transform base_transform = base_to_sensor * pose
			* base_to_sensor.inverse();

//	nav_msgs::Odometry odometry_msg;
//	odometry_msg.header.stamp = query_timestamp;
//	odometry_msg.header.frame_id = getOdometerReferenceFrame();
//	odometry_msg.child_frame_id = getRobotFrame();
//	tf::poseTFToMsg(base_transform, odometry_msg.pose.pose);
//
//	// calculate twist (not possible for first run as no delta_t can be computed)
//	tf::Transform delta_base_transform = base_to_sensor * motion
//			* base_to_sensor.inverse();
//
//	double delta_t = (query_timestamp - train_timestamp).toSec();
//	odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX()
//			/ delta_t;
//	odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY()
//			/ delta_t;
//	odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ()
//			/ delta_t;
//	double yaw, pitch, roll;
//	delta_base_transform.getBasis().getEulerYPR(yaw, pitch, roll);
//	odometry_msg.twist.twist.angular.x = roll / delta_t;
//	odometry_msg.twist.twist.angular.y = pitch / delta_t;
//	odometry_msg.twist.twist.angular.z = yaw / delta_t;
//
////	odometry_msg.pose.covariance = pose_covariance_;
////	odometry_msg.twist.covariance = twist_covariance_;
//	odometry_advertiser.publish(odometry_msg);
//
//	geometry_msgs::PoseStamped pose_msg;
//	pose_msg.header.stamp = odometry_msg.header.stamp;
//	pose_msg.header.frame_id = odometry_msg.header.frame_id;
//	pose_msg.pose = odometry_msg.pose.pose;
//
//	pose_advertiser.publish(pose_msg);

	odom_broadcaster.sendTransform(
			tf::StampedTransform(base_transform, query_timestamp,
					getOdometerReferenceFrame(), getRobotFrame()));

//	//next, we'll publish the odometry message over ROS
//	odometry.header.stamp = ros::Time::now();
//	odometry.header.frame_id = getOdometerReferenceFrame();
//	odometry.child_frame_id = getSensorFrame();
//
//	tf::poseTFToMsg(base_transform, odometry.pose.pose);
//
//	double delta_t = 1.0; //(query_timestamp-train_timestamp).toSec();
//	odometry.twist.twist.linear.x = motion.getOrigin().getX() / delta_t;
//	odometry.twist.twist.linear.y = motion.getOrigin().getY() / delta_t;
//	odometry.twist.twist.linear.z = motion.getOrigin().getZ() / delta_t;
//	double yaw, pitch, roll;
//	motion.getBasis().getEulerYPR(yaw, pitch, roll);
//	odometry.twist.twist.angular.x = roll / delta_t;
//	odometry.twist.twist.angular.y = pitch / delta_t;
//	odometry.twist.twist.angular.z = yaw / delta_t;
//
//	odometry_advertiser.publish(odometry);

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
	std::vector<char> inliers = mot_proc.getInlierMask();
	ImageProcessor::draw_feature(query_image->image, feature_image.image,
			query_kpts);
	return 0;
}

int MonoOdometer::drawMatchesImage()
{
	matches_image.encoding = sensor_msgs::image_encodings::BGR8;
	std::vector<char> inliers = mot_proc.getInlierMask();
	inliers = inliers.size() == matches.size() ? inliers : std::vector<char>();

	ImageProcessor::draw_matches(query_image->image, query_kpts,
			train_image->image, train_kpts, matches, matches_image.image,
			inliers);
	return 0;
}

int MonoOdometer::drawOptFlowImage()
{
	optflow_image.encoding = sensor_msgs::image_encodings::BGR8;

	std::vector<char> inliers = mot_proc.getInlierMask();
	inliers = inliers.size() == matches.size() ? inliers : std::vector<char>();

	ImageProcessor::draw_optflow(query_image->image, optflow_image.image,
			query_kpts, train_kpts, matches, inliers);

	return 0;
}

int MonoOdometer::drawDisplacementImage()
{
	displacement_image.encoding = sensor_msgs::image_encodings::BGR8;

	std::vector<char> inliers = mot_proc.getInlierMask();
	inliers = inliers.size() == matches.size() ? inliers : std::vector<char>();

	ImageProcessor::draw_displacement(query_image->image,
			displacement_image.image, query_kpts, train_kpts, matches, inliers);

	return 0;
}

/**
 * 	@brief The Image Callback method is responsible for handling the income
 * 	messages from the camera device and extract the frame features and
 * 	descriptors.
 *
 * 	 @param[in]	msg		Income message from the defined camera topic.
 */
void MonoOdometer::CallbackHandler(const sensor_msgs::ImageConstPtr& img,
		const sensor_msgs::CameraInfoConstPtr& cam =
				sensor_msgs::CameraInfoConstPtr())
{
	query_timestamp = img->header.stamp;
	convertSensorMsgToImage(img, query_image);

	if (find_optflow_libviso())	// Choose between optflow or rich matching
	{
		ROS_DEBUG("Matched features: %d", matches.size());

		cv::Mat A = cv::abs(cv::Mat(train_pts) - cv::Mat(query_pts));
		/*
		 * displacement verifies wether there was a great displacement
		 * between the features in two consecutives frames
		 */
//		double displacement = cv::sum(A).val[0] / train_pts.size();
//		ROS_INFO("Sum of all displacements: %f", displacement);
		if (/*(displacement > 10) &&*/(matches.size() > 8))
		{
			/**
			 * @todo The only parameters to the estimate_motion() should be the train
			 * and query points. K should be passed on Motion Processor construction.
			 * The points are already matched, therefore there's no need for the
			 * matches parameter.
			 */
			mot_proc.estimate_motion(train_pts, query_pts, matches, K);
		}
		else
		{
			/**
			 * @todo What happens when it's not possible to estimate a motion? Kalman?
			 * For now, the no motion politicy was taken. So meaning the robot stand
			 * still for a frame. May cause some mayheim on the next frame due the large
			 * displacement caused by the one-frame-gap.
			 */
			mot_proc.noMotion();
		}

		/*
		 * Sets the new pose.
		 */
		update_pose();
	}
	if (matches.size() > 8)
	{
		if (output_feature_advertiser.getNumSubscribers() > 0)
		{
			drawFeatureImage();
			output_feature_advertiser.publish(feature_image.toImageMsg());
		}
		if (output_matches_advertiser.getNumSubscribers() > 0)
		{
			drawMatchesImage();
			output_matches_advertiser.publish(matches_image.toImageMsg());
		}
		if (output_optflow_advertiser.getNumSubscribers() > 0)
		{
			drawOptFlowImage();
			output_optflow_advertiser.publish(optflow_image.toImageMsg());
		}
		if (output_displacement_advertiser.getNumSubscribers() > 0)
		{
			drawDisplacementImage();
			output_displacement_advertiser.publish(
					displacement_image.toImageMsg());
		}
	}

	train_kpts.clear();
	train_kpts = query_kpts;
	train_desc = query_desc;
	train_image = query_image;
	train_timestamp = query_timestamp;

}

void MonoOdometer::ImageCallback(const sensor_msgs::ImageConstPtr& img)
{
	cv::FileStorage calib_data;
	calib_data.open(getIntrinsicMatrixPath().c_str(), cv::FileStorage::READ);
	if (calib_data.isOpened())
	{
		calib_data["cameraMatrix"] >> K;
	}
	else
	{
		ROS_ERROR("Could not open file %s", getIntrinsicMatrixPath().c_str());
	}

	CallbackHandler(img);
}

void MonoOdometer::CameraCallback(const sensor_msgs::ImageConstPtr& img,
		const sensor_msgs::CameraInfoConstPtr& cam)
{
	K = cv::Mat(3, 3, CV_64F, (double*) cam->K.elems);

	CallbackHandler(img, cam);
}

} /* namespace LRM */
