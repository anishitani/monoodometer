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
 * Later this class should be moved to convenient file.
 */
class Tracker
{
protected:
	Tracker()
	{
	}
	virtual ~Tracker()
	{
	}
};

class ESM: public Tracker
{

public:
	ESM()
	{
		dof = 3; //Default number of degrees of freedom
		initESM();

		// @todo Veh2Cam should be a parameter

		// Camera Pitch
		float rx = -0.03;
		cv::Mat RotX = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, cos(rx), sin(
				rx), 0, 0, -sin(rx), cos(rx), 0, 0, 0, 0, 1);

		cv::Mat(RotX).copyTo(Veh2Cam);
	}

	ESM(int DoF)
	{
		dof = DoF;
		initESM();
	}

	~ESM()
	{

	}

	void minSSDSE2Motion(cv::Mat TIRef, cv::Mat ICur, float width, float height,
			cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T);
	void minSSDSE3Motion(cv::Mat TIRef, cv::Mat ICur, float width, float height,
			cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T);

private:

	/**
	 * \brief Number of degrees of freedom of the vehicle motion.
	 */
	int dof;

	/**
	 * \brief Algebra Lie Generators.
	 */
	std::vector<cv::Mat> A;

	/**
	 * \brief Tranformation from vehicle coordinate system
	 * to camera coordinate system.
	 */
	cv::Mat Veh2Cam;

	void initESM();

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

