/*
 * main.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: nishitani
 */

#include <ros/ros.h>

#include "mono_odometer.h"

using namespace LRM;

/**
 *	main()
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "mono_odometer");
	std::string node_name = ros::this_node::getName();

	ros::NodeHandle nh(node_name);

	image_transport::ImageTransport it( nh );

	MonoOdometer odometer( nh,it );

	image_transport::Subscriber sub = it.subscribe("/camera/image",1,&MonoOdometer::ImageCallback,&odometer);

	ros::spin();

}

