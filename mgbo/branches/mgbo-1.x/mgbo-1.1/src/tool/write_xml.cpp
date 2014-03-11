/*
 * write_xml.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: nishitani
 */

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#define CONFIG_PATH "config"

int main (int argc, char **argv)
{
	cv::FileStorage fs(
			"config/lrm.xml",
			cv::FileStorage::WRITE);

	if(!fs.isOpened())
	{
		fprintf(stderr, "Failed to open XML file %s!", argv[1]);
		return -1;
	}

	fs << "text" << "value";
	fs << "numb" << 10.1;

	fs.release();

	printf("ACABOU!\n");
	getchar();

	return 0;
}

