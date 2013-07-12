/*
 * frame.cpp
 *
 *  Created on: Sep 25, 2012
 *      Author: nishitani
 */

#include "frame.h"

namespace LRM {

/**
 * Frame Constructor:
 * 		Instantiates a new frame. The Frame class subscribes a frame from
 * 	 a camera and process it identifying features and descriptors. Also
 * 	 responsible for the match of the features over other frames.
 *
 * 	 @param[in]	nh	Node handle of the node of the monocular odometer.
 */
Frame::Frame( const sensor_msgs::ImageConstPtr& msg ) {
	/// TODO: Deal with different image encodings
	img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	timestamp = msg->header.stamp;
//	this->image = img_ptr->image;
}

Frame::~Frame() {
	// TODO Auto-generated destructor stub
}

/**
 * init:
 *		Method responsible for the first detection, extraction and match of features.
 *
 * 	 @param[in]	fh	Retainer of the features hanndling methods.
 */
void Frame::init( boost::shared_ptr<FeatureHandler> fh ) {

	fh->detect( img_ptr->image,featList );
	fh->extract( img_ptr->image,featList,featDescriptors );
}

/**
 * process:
 *		Method responsible for detection, extraction and match of features.
 *
 * 	 @param[in]	fh					Retainer of the features hanndling methods.
 * 	 @param[in] lastFrame			Last processed frame.
 */
void Frame::process( boost::shared_ptr<FeatureHandler> fh, Frame lastFrame ) {

	fh->detect( img_ptr->image,featList );
	fh->extract( img_ptr->image,featList,featDescriptors );
	fh->match( featDescriptors,lastFrame.getFeatDescriptors(), featList, lastFrame.getFeatList(), featMatches );

}

/**
 * markFeatures:
 *		Mark the features in the original image.
 *
 */
void Frame::markFeatures() {
	cv::drawKeypoints( img_ptr->image,featList,img_ptr->image,CV_RGB(0,255,0),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS|cv::DrawMatchesFlags::DRAW_OVER_OUTIMG );
}

/**
 * markTracks:
 *		Mark the features track in the image
 *
 * 	 @param[in]	lastFeatList	The list of features in the last frame
 */
void Frame::markTracks( std::vector<cv::KeyPoint> lastFeatList ) {
	for(uint i=0 ; i<featMatches.size() ; i++){
        cv::Point2f pt_new = featList[featMatches[i].queryIdx].pt;
        cv::Point2f pt_old = lastFeatList[featMatches[i].trainIdx].pt;

        cv::line( 	img_ptr->image	, pt_old, pt_new, 	cv::Scalar(0, 0, 255), 1 );
	}
}

} /* namespace LRM */
