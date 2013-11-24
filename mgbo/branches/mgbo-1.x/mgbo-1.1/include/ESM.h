/*
 * ESM.h
 *
 *  Created on: Sep 10, 2013
 *      Author: nishitani
 */

#ifndef ESM_H_
#define ESM_H_

#include <cstdio>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include <math_tools.h>

/*
 * Tracker Class
 * Later this class should be moved to an appropriate file.
 */
class Tracker
{
protected:
	Tracker()
	{
	}
	virtual ~Tracker(){}
};

class ESM: public Tracker
{

public:
	ESM()
	{
		dof = 3; //Default number of degrees of freedom
		Veh2Cam = cv::Mat::eye(4, 4, CV_32F);
	}

	ESM(int DoF)
	{
		dof = DoF;
		Veh2Cam = cv::Mat::eye(4, 4, CV_32F);
	}

	~ESM()
	{

	}

	void minSSDSE2Motion(cv::Mat TIRef, cv::Mat ICur, float width, float height,
			cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T);
	void minSSDSE3Motion(cv::Mat TIRef, cv::Mat ICur, float width, float height,
				cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T);

private:

	int dof;	// Motion degree of freedom

	/*
	 * Lie Algebra Generators
	 */
	std::vector<cv::Mat> A;

	/*
	 * Vehicle to Camera transformation
	 */
	cv::Mat Veh2Cam;

	/*
	 * Planar Motion SE(2)
	 */
	bool updateSSDSE2Motion(cv::Mat TIRef, cv::Mat ICur, float width,
			float height, cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T,
			float &RMS);
	cv::Mat imgJacSE2planar(cv::Mat mIx, cv::Mat mIy, float width, float height,
			cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T);

	/*
	 * 6-DoF Motion SE(3)
	 */
	bool updateSSDSE3Motion(cv::Mat TIRef, cv::Mat ICur, float width,
			float height, cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T,
			float &RMS);
	cv::Mat imgJacSE3planar(cv::Mat mIx, cv::Mat mIy, float width, float height,
			cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T);

	/*
	 * RMS generator via Sub Sample.
	 */
	void subSample(int num, cv::Mat TIRef, cv::Mat ICur, float width,
			float height, cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T);
};

#endif /* ESM_H_ */

