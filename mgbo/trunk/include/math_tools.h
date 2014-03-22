/*
 * math_tools.h
 *
 *  Created on: Nov 23, 2013
 *      Author: nishitani
 */

#ifndef MATH_TOOLS_H_
#define MATH_TOOLS_H_

#include <climits>
#include <opencv2/opencv.hpp>
#include <expm.hpp>

void gradient(cv::Mat I, cv::Mat &Ix, cv::Mat &Iy);
void meshgrid(int x0, int y0, int width, int height, cv::Mat &gridx,
		cv::Mat &gridy);
cv::Mat skew(cv::Mat a);
cv::Mat expm(cv::Mat x);
bool IsNumber(double x);
bool IsFiniteNumber(double x);

#endif /* MATH_TOOLS_H_ */
