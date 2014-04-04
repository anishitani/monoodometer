/*
 * math_tools.cpp
 *
 *  Created on: Nov 23, 2013
 *      Author: nishitani
 */

#ifndef MATH_TOOLS_CPP_
#define MATH_TOOLS_CPP_

#include <math_tools.h>

void gradient(cv::Mat I, cv::Mat &Ix, cv::Mat &Iy)
{
	cv::Matx13f dx(-0.5, 0, 0.5);
	cv::Matx31f dy(-0.5, 0, 0.5);

	cv::filter2D(I, Ix, CV_32F, dx);
	cv::filter2D(I, Iy, CV_32F, dy);

	cv::subtract(I.col(1), I.col(0), Ix.col(0));
	cv::subtract(I.col(I.cols - 1), I.col(I.cols - 2), Ix.col(I.cols - 1));

	cv::subtract(I.row(1), I.row(0), Iy.row(0));
	cv::subtract(I.row(I.rows - 1), I.row(I.rows - 2), Iy.row(I.rows - 1));
}

void meshgrid(int x0, int y0, int width, int height, cv::Mat &gridx,
		cv::Mat &gridy)
{
	cv::Mat rowx(1, width, CV_32F), coly(height, 1, CV_32F);
	for (int i = 0; i < width; i++)
		rowx.at<float>(0, i) = x0 + i;
	for (int i = 0; i < height; i++)
		coly.at<float>(i, 0) = y0 + i;

	cv::repeat(rowx, height, 1, gridx);
	cv::repeat(coly, 1, width, gridy);
}

cv::Mat skew(cv::Mat a)
{
	return (cv::Mat_<float>(3, 3) << 0, -a.at<float>(2), a.at<float>(1), a.at<
			float>(2), 0, -a.at<float>(0), -a.at<float>(1), a.at<float>(0), 0);
}

cv::Mat expm(cv::Mat x)
{
	assert(x.rows == x.cols);

	boost::numeric::ublas::matrix<float> h(x.rows, x.cols);
	for (int i = 0; i < x.rows; i++)
	{
		for (int j = 0; j < x.cols; j++)
		{
			h(i, j) = x.at<float>(i, j);
		}
	}

	h = expm_pad(h);

	cv::Mat A(x.rows, x.cols, CV_32F);
	for (int i = 0; i < x.rows; i++)
	{
		for (int j = 0; j < x.cols; j++)
		{
			A.at<float>(i, j) = h(i, j);
		}
	}

	return A;
}

bool IsNumber(double x)
{
	// This looks like it should always be true,
	// but it's false if x is a NaN.
	return (x == x);
}

bool IsFiniteNumber(double x)
{

	return (x <= DBL_MAX && x >= -DBL_MAX);
}

#endif /* MATH_TOOLS_CPP_ */
