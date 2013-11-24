/*
 * ESM.cpp
 *
 *  Created on: Nov 23, 2013
 *      Author: nishitani
 */

#include <ESM.h>

void ESM::minSSDSE2Motion(cv::Mat TIRef, cv::Mat ICur, float width, float height,
		cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T)
{
	A = std::vector<cv::Mat>(dof);
	A[0] =
			(cv::Mat_<float>(4, 4) << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	A[1] =
			(cv::Mat_<float>(4, 4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);
	A[2] =
			(cv::Mat_<float>(4, 4) << 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0);

	int iter = 50;
	float RMS;

	float bestRMS = 100000;	// Infinity
	cv::Mat TOld(T), TNew;
	cv::Mat TBest;

	int i;
	for (i = 0; i < iter; i++)
	{
		TNew = TOld.clone();
		bool converged = updateSSDSE2Motion(TIRef, ICur, width, height, K,
				norVec, G0, TNew, RMS);

		if (RMS < bestRMS)
		{
			printf("RMS: %f, bestRMS: %f\n", RMS, bestRMS);
			bestRMS = RMS;
			TOld.copyTo(TBest);
		}
		else
		{
//			printf("RMS: %f\n", RMS);
		}

		if (converged)
			break;
		TOld = TNew;
	}
//	printf("%d iterations\n", i);

	T = TBest;
	RMS = bestRMS;

}

bool ESM::updateSSDSE2Motion(cv::Mat TIRef, cv::Mat ICur, float width, float height,
		cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T, float &RMS)
{
	/// @todo Constants K and K^-1

	cv::Mat R(T, cv::Rect(0, 0, 3, 3)), t(T, cv::Rect(3, 0, 1, 3));
	cv::Mat G = K * (R - t * norVec.t()) * K.inv();

	cv::Mat mask;
	cv::Mat di(TIRef.rows, TIRef.cols, CV_32F, cv::Scalar(0));
	cv::Mat TICur;

	/*
	 * Warps the current image fitting the reference ROI.
	 * Points outside the image are set to -1.
	 */
	cv::warpPerspective(ICur, TICur, G * G0, cv::Size(width, height),
			cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT,
			cv::Scalar(-1));

	/// @todo What happens with the points out of the mask?
	mask = cv::Mat::ones(TICur.rows, TICur.cols, CV_8U);
	for (int i = 0; i < TICur.rows * TICur.cols; i++)
		if (TICur.at<uchar>(i) < 0)
			mask.at<uchar>(i) = 0;
	cv::subtract(TIRef, TICur, di, mask, CV_32F);

	/* **********************************
	 * BEGIN TESTING
	 * **********************************/
	cv::Mat diff(TIRef.rows, 3 * TIRef.cols, CV_8U);

	cv::Rect win1(0, 0, TIRef.cols, TIRef.rows);
	cv::Rect win2(TIRef.cols, 0, TIRef.cols, TIRef.rows);
	cv::Rect win3(2 * TIRef.cols, 0, TIRef.cols, TIRef.rows);

	TIRef.copyTo(diff(win1));
	TICur.copyTo(diff(win2));
	di.copyTo(diff(win3));

	char frase[256];
	sprintf(frase, "RMS: %f", RMS);
	cv::putText(diff, frase, cv::Point(3, 10), cv::FONT_HERSHEY_SIMPLEX, 0.3,
			cv::Scalar(255));

	cv::imshow("diff", diff);
	char key = cv::waitKey(10);
	if (key == 'q')
		exit(0);
	/* **********************************
	 * END TESTING
	 * **********************************/

	// Root mean square error calculation
	cv::Mat di2;
	cv::pow(di, 2, di2);
	RMS = (float) cv::mean(di2, mask).val[0];
	RMS = std::sqrt(RMS);

	// Gradient of Reference and Current images
	cv::Mat dxRef, dyRef;
	cv::Mat dxCur, dyCur;

	// Gradient operation as defined in MATLAB
	gradient(TIRef, dxRef, dyRef);
	gradient(TICur, dxCur, dyCur);

	cv::Mat J = imgJacSE2planar((dxRef + dxCur) / 2, (dyRef + dyCur) / 2, width,
			height, K, norVec, G0, T);

	cv::Mat Jinv = (J.t() * J).inv() * J.t();

	di = di.t();
	di = di.reshape(1, width * height);

	cv::Mat d = Jinv * di;

	if (cv::norm(d) < 1e-5)
		return 1;

	cv::Mat xA(4, 4, CV_32F, cv::Scalar(0));
	for (int i = 0; i < (int) A.size(); i++)
		xA += d.at<float>(i) * A[i];
	cv::Mat dT = expm(Veh2Cam * xA);

	T = T * dT;

	return 0;
}

cv::Mat ESM::imgJacSE2planar(cv::Mat mIx, cv::Mat mIy, float width, float height,
		cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T)
{
	cv::Mat Jesm;

	/// @todo Constants T{C,V} and T{C,V}^-1

	cv::Mat K4 = cv::Mat::zeros(4, 4, CV_32F);
	K.copyTo(K4(cv::Rect(0, 0, 3, 3)));
	cv::Mat K4Inv = cv::Mat::zeros(4, 4, CV_32F);
	cv::Mat(K.inv()).copyTo(K4Inv(cv::Rect(0, 0, 3, 3)));

	cv::Mat norMat = cv::Mat::zeros(4, 4, CV_32F);
	norMat(cv::Rect(0, 0, 3, 3)) = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat(-norVec.t()).copyTo(norMat(cv::Rect(0, 3, 3, 1)));

	cv::Mat JTx(9, A.size(), CV_32F);
	for (int i = 0; i < (int) A.size(); i++)
	{
		cv::Mat JTxi = K4 * Veh2Cam * T * A[i] * Veh2Cam.t() * norMat * K4Inv;
		JTxi(cv::Rect(0, 0, 3, 3)).clone().reshape(1, 9).copyTo(JTx.col(i));
	}

	cv::Mat dx = cv::Mat(mIx.t()).reshape(1, 1).t();
	cv::Mat dy = cv::Mat(mIy.t()).reshape(1, 1).t();

	cv::Mat px, py;
	meshgrid(G0.at<float>(0, 2), G0.at<float>(1, 2), width, height, px, py);
	px = cv::Mat(px.t()).reshape(1, 1).t();
	py = cv::Mat(py.t()).reshape(1, 1).t();

	cv::Mat JIW(width * height, 9, CV_32F);
	/*
	 * The first three equations are dx*p.
	 * The second triple are dy*p.
	 * The last triple are -(dx*px+dy*py)*p
	 */
	cv::multiply(dx, px, JIW.col(0));
	cv::multiply(dx, py, JIW.col(1));
	dx.copyTo(JIW.col(2));

	cv::multiply(dy, px, JIW.col(3));
	cv::multiply(dy, py, JIW.col(4));
	dy.copyTo(JIW.col(5));

	cv::multiply(-(JIW.col(0) + JIW.col(4)), px, JIW.col(6));
	cv::multiply(-(JIW.col(0) + JIW.col(4)), py, JIW.col(7));
	cv::Mat(-JIW.col(0) - JIW.col(4)).copyTo(JIW.col(8));

	Jesm = JIW * JTx;

	return Jesm;
}

void ESM::minSSDSE3Motion(cv::Mat TIRef, cv::Mat ICur, float width, float height,
		cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T)
{
	int iter = 50;
	float RMS;

	float bestRMS = 100000;	// Infinity
	cv::Mat TOld(T), TNew;
	cv::Mat TBest;

	int i;
	for (i = 0; i < iter; i++)
	{
		TNew = TOld.clone();
		bool converged = updateSSDSE3Motion(TIRef, ICur, width, height, K,
				norVec, G0, TNew, RMS);

		if (RMS < bestRMS)
		{
			printf("RMS: %f, bestRMS: %f\n", RMS, bestRMS);
			bestRMS = RMS;
			TOld.copyTo(TBest);
		}
		else
		{
			printf("RMS: %f\n", RMS);
		}

		if (converged)
			break;
		TOld = TNew;
	}
	printf("%d iterations\n", i);

	T = TBest;
	RMS = bestRMS;
}

bool ESM::updateSSDSE3Motion(cv::Mat TIRef, cv::Mat ICur, float width, float height,
		cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T, float &RMS)
{
	/// @todo Constants K and K^-1

	cv::Mat R(T, cv::Rect(0, 0, 3, 3)), t(T, cv::Rect(3, 0, 1, 3));
	cv::Mat G = K * (R + t * norVec.t()) * K.inv();

	cv::Mat Kh = cv::Mat::eye(4, 4, CV_32F);
	K.copyTo(Kh(cv::Rect(0, 0, 3, 3)));

	cv::Mat mask;
	cv::Mat di;
	cv::Mat TICur;
//	ICur.convertTo(TICur, CV_32F);

	/*
	 * Warps the current image fitting the reference ROI.
	 * Points outside the image are set to -1.
	 */
	cv::warpPerspective(ICur, TICur, G * G0, cv::Size(width, height),
			cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT,
			cv::Scalar(-1));

	mask = cv::Mat::ones(TICur.rows, TICur.cols, CV_8U);
	for (int i = 0; i < TICur.rows * TICur.cols; i++)
		if (TICur.data[i] < 0)
			mask.data[i] = 0;
	cv::subtract(TIRef, TICur, di, mask, CV_64F);

	// Root mean square error calculation
	cv::Mat di2;
	cv::pow(di, 2, di2);
	RMS = (float) cv::mean(di2, mask).val[0];
	RMS = std::sqrt(RMS);

	/* **********************************
	 * BEGIN TESTING
	 * **********************************/

	cv::Mat diff(TIRef.rows, 3 * TIRef.cols, CV_8U);

	cv::Rect win1(0, 0, TIRef.cols, TIRef.rows);
	cv::Rect win2(TIRef.cols, 0, TIRef.cols, TIRef.rows);
	cv::Rect win3(2 * TIRef.cols, 0, TIRef.cols, TIRef.rows);

	TIRef.copyTo(diff(win1));
	TICur.copyTo(diff(win2));
	di.copyTo(diff(win3));

	char frase[256];
	sprintf(frase, "RMS: %f", RMS);
	cv::putText(diff, frase, cv::Point(3, 10), cv::FONT_HERSHEY_SIMPLEX, 0.3,
			cv::Scalar(255));

	cv::imshow("diff", diff);
	char key = cv::waitKey(1000);
	if (key == 'q')
		exit(0);
	/* **********************************
	 * END TESTING
	 * **********************************/

	// Gradient of Reference and Current images
	cv::Mat dxRef, dyRef;
	cv::Mat dxCur, dyCur;

	// Gradient operation as defined on MATLAB
	gradient(TIRef, dxRef, dyRef);
	gradient(TICur, dxCur, dyCur);

	cv::Mat J = imgJacSE3planar((dxRef + dxCur) / 2, (dyRef + dyCur) / 2, width,
			height, K, norVec, G0, T);

	J.convertTo(J, CV_64F);
	cv::Mat Jinv = (J.t() * J).inv() * J.t();

	di.convertTo(di, CV_64F);
	di = di.t();
	di = di.reshape(1, width * height);
	cv::Mat d = Jinv * di;

	cv::Mat xA =
			(cv::Mat_<float>(4, 4) << 0, -d.at<double>(5), d.at<double>(4), d.at<
					double>(0), d.at<double>(5), 0, -d.at<double>(3), d.at<
					double>(1), -d.at<double>(4), d.at<double>(3), 0, d.at<
					double>(2), 0, 0, 0, 0);

	cv::Mat dT = expm(xA);
	T = T * dT;

	if (cv::norm(d) < 1e-5)
		return 1;

	return 0;
}

cv::Mat ESM::imgJacSE3planar(cv::Mat mIx, cv::Mat mIy, float width, float height,
		cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T)
{
	cv::Mat J;

	cv::Mat R(T, cv::Rect(0, 0, 3, 3)), t(T, cv::Rect(3, 0, 1, 3));
	cv::Mat t0 = R.t() * t * (1 / (1 + (norVec.t() * R.t() * t)));

	float t0x = t0.at<float>(0);
	float t0y = t0.at<float>(1);
	float t0z = t0.at<float>(2);

	cv::Mat x, y;
	meshgrid(1, 1, width, height, x, y);
	x = x.t();
	x = x.reshape(1, 1);
	y = y.t();
	y = y.reshape(1, 1);

	cv::Mat Ix = mIx.t();
	Ix = Ix.reshape(1, 1);
	cv::Mat Iy = mIy.t();
	Iy = Iy.reshape(1, 1);

	float n1 = norVec.at<float>(0);
	float n2 = norVec.at<float>(1);
	float n3 = norVec.at<float>(2);

	float f = K.at<float>(0, 0);
	float u0 = K.at<float>(0, 2);
	float v0 = K.at<float>(1, 2);

	cv::Mat px = (u0 - x - G0.at<float>(0, 2)) / f;
	cv::Mat py = (v0 - y - G0.at<float>(1, 2)) / f;

	cv::Mat Ax = t0x + px * t0z;
	cv::Mat Ay = t0y + py * t0z;
	cv::Mat Az = -f * (n1 * px + n2 * py - n3);

	J.push_back(
			cv::Mat(
					(-n1 * Ax + 1).mul(Az).mul(Ix)
							+ (-n1 * Ay).mul(Az).mul(Iy)));
	J.push_back(
			cv::Mat(
					(-n2 * Ax).mul(Az).mul(Ix)
							+ (-n2 * Ay + 1).mul(Az).mul(Iy)));
	J.push_back(
			cv::Mat(
					(-n3 * Ax + px).mul(Az).mul(Ix)
							+ (-n3 * Ay + py).mul(Az).mul(Iy)));

	cv::Mat Bx = n1 + n3 * px;
	cv::Mat By = n2 + n3 * py;
	cv::Mat Bxy = n2 * px - n1 * py;
	float Bz = t0z * n3 - 1;

	J.push_back(
			cv::Mat(
					f * (t0x * By + n2 * t0z * px + Bz * px.mul(py)).mul(Ix)
							+ f
									* (t0y * By + n2 * t0z * py - 1
											+ Bz * py.mul(py)).mul(Iy)));
	J.push_back(
			cv::Mat(
					-f
							* (t0x * Bx + n1 * t0z * px + Bz * px.mul(px) - 1).mul(
									Ix)
							+ -f
									* (t0y * Bx + n1 * t0z * py
											+ Bz * px.mul(py)).mul(Iy)));
	J.push_back(
			cv::Mat(
					f * (t0x * Bxy + py + t0z * Bxy.mul(px)).mul(Ix)
							+ f
									* (t0y * Bxy - px + t0z * Bxy.mul(py)).mul(
											Iy)));

	return J.t();
}

void ESM::subSample(int num, cv::Mat TIRef, cv::Mat ICur, float width,
		float height, cv::Mat K, cv::Mat norVec, cv::Mat G0, cv::Mat &T)
{
	char filename[256];
	sprintf(filename, "data/rms/rms%d.txt", num);
	FILE *f;
	f = fopen(filename, "w");

	float factor = 0.1;

	for (float dx = -2; dx < 2; dx += factor)
	{
		for (float dz = -2; dz < 2; dz += factor)
		{
			cv::Mat R(T, cv::Rect(0, 0, 3, 3)), t;
			T(cv::Rect(3, 0, 1, 3)).copyTo(t);
			t.at<float>(0) += dx;
			t.at<float>(2) += dz;
			cv::Mat G = K * (R - t * norVec.t()) * K.inv();

			cv::Mat mask;
			cv::Mat di;
			cv::Mat TICur;

			/*
			 * Warps the current image fitting the reference ROI.
			 * Points outside the image are set to -1.
			 */
			cv::warpPerspective(ICur, TICur, G * G0, cv::Size(width, height),
					cv::INTER_LINEAR + cv::WARP_INVERSE_MAP,
					cv::BORDER_CONSTANT, cv::Scalar(-1));

			mask = cv::Mat::ones(TICur.rows, TICur.cols, CV_8U);
			for (int i = 0; i < TICur.rows * TICur.cols; i++)
				if (TICur.data[i] < 0)
					mask.data[i] = 0;
			cv::subtract(TIRef, TICur, di, mask, CV_32F);

			// Root mean square error calculation
			cv::Mat di2;
			cv::pow(di, 2, di2);
			float RMS = (float) cv::mean(di2, mask).val[0];
			RMS = std::sqrt(RMS);

			fprintf(f, "%f %f %f\n", t.at<float>(0), t.at<float>(2), RMS);

			/*
			 * BEGIN TESTING
			 */
			cv::Mat diff(TIRef.rows, 3 * TIRef.cols, CV_8U);

			cv::Rect win1(0, 0, TIRef.cols, TIRef.rows);
			cv::Rect win2(TIRef.cols, 0, TIRef.cols, TIRef.rows);
			cv::Rect win3(2 * TIRef.cols, 0, TIRef.cols, TIRef.rows);

			TIRef.copyTo(diff(win1));
			TICur.copyTo(diff(win2));
			cv::Mat(TICur - TIRef).copyTo(diff(win3));

//			cv::imshow("diff", diff);
//			char key = cv::waitKey(60);
//			if (key == 'q')
//				exit(0);
			/*
			 * END TESTING
			 */
		}
	}

	fclose(f);
}
