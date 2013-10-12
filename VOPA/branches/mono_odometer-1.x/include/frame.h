/*
 * frame.h
 *
 *  Created on: Sep 25, 2012
 *      Author: nishitani
 */

#ifndef FRAME_H_
#define FRAME_H_

#include "core.h"
#include "feature.h"

namespace LRM {



/**
 *	Class Frame:
 *		The class Frame is responsible to retain the information about a frame
 *	of the data stream. Also responsible of possible alterations on the image.
 */
class Frame {
public:
	// Constructor and Destructor
	Frame( const sensor_msgs::ImageConstPtr& msg );
	virtual ~Frame();

	// Public methods
	void init( boost::shared_ptr<FeatureHandler> fh );
	void process( boost::shared_ptr<FeatureHandler> fh, Frame lastFrame );
	void markFeatures();
	void markTracks( std::vector<cv::KeyPoint> lastFeatList );

	sensor_msgs::ImagePtr 		getImageMsg(){ return img_ptr->toImageMsg(); }
	int 						getNumberOfFeatures(){ return featList.size(); }
	std::vector<cv::KeyPoint>	getFeatList(){ return featList; }
	cv::Mat						getFeatDescriptors(){ return featDescriptors; }
	std::vector<cv::DMatch> 	getFeatMatches(){ return featMatches; }
	ros::Time					getTimestamp(){ return timestamp; }

private:
	std::vector<cv::KeyPoint>	featList;			///< Detected features' list
	cv::Mat						featDescriptors;	///< Detected features' descriptor
	std::vector<cv::DMatch> 	featMatches;		///< Detected features' matches

	cv_bridge::CvImagePtr img_ptr;			///< Pointer to the frame image
	ros::Time timestamp;					///< Timestamp of the frame

	// Private methods

};

} /* namespace LRM */
#endif /* FRAME_H_ */
