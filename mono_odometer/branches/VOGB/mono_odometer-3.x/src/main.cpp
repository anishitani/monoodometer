/*
 * main.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

/*! \mainpage Mono Odometer Package Index Page
 *
 * \section intro_sec Introduction
 *
 * This is odometry package. The Mono Odometer package process monocular images
 * and estimate the motion.
 *
 * This is a ROS package.
 *
 * \section usage_sec Usage
 *
 * The package can run without setting parameters. Default parameter configuration
 * will be used.
 *
 * The use of a launch is recommended as a tool for parameter setting.
 */

#include <ros/ros.h>

#include "mono_odometer.h"

using namespace LRM;

/**
 *	main()
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mono_odometer");
	std::string node_name = ros::this_node::getName();

	ros::NodeHandle nh(node_name);

	image_transport::ImageTransport it(nh);

	MonoOdometer odometer(nh, it);

	ros::spin();

}

