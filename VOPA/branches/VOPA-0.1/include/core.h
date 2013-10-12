/*
 * core.h
 *
 *  Created on: Oct 11, 2012
 *      Author: nishitani
 */

#ifndef CORE_H_
#define CORE_H_

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <boost/smart_ptr.hpp>

#include <algorithm>
#include <vector>
#include <deque>

class FeatureHandler;
class Frame;

#endif /* CORE_H_ */
