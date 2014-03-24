/*
 * error_map.cpp
 *
 *  Created on: Mar 1, 2014
 *      Author: nishitani
 */

/*
 * @todo For each run, the resultant maps should be stored on
 * a separated folder
 * Each folder must contain a description file defining the
 * parameters used for the maps generations
 * The parameters are:
 * 	> height: Camera height w.r.t. the ground
 * 	> pitch: Camera pitch angle
 * 	> POSX: Upper-left corner X coordinate
 * 	> POSY: Upper-left corner Y coordinate
 * 	> SIZX: ROI width
 * 	> SIZY: ROI height
 * 	> disp_x: Displacement in X of the current ROI w.r.t. the last ROI
 * 	> disp_y: Displacement in Y of the current ROI w.r.t. the last ROI
 * 	> dt: Error map granularity
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <tclap/CmdLine.h>

#include <opencv2/opencv.hpp>

#define WINDOW_NAME "ERROR MAP"
#define ROI_WINDOW_NAME "ROI WINDOW"

/**
 * Generates the error maps calculating the difference
 * between the current and the last images.
 *
 * @param curr Current image
 * @param last Last image
 * @param H Transformation from the current to the last frame
 * @param map Resultant map
 */
void process_map(cv::Mat curr, cv::Mat last, cv::Mat H, cv::Mat &map);

/**
 * Applies the transformation aHb, which transforms the image on
 * the A reference frame to the B reference frame
 *
 * @param Ia Image on the reference frame A
 * @param Ib Image on the reference frame B
 * @param aHb Transformation from the frame A to frame B
 */
void change_reference(cv::Mat Ia, cv::Mat &Ib, cv::Mat aHb);

int main(int argc, char **argv)
{
	std::string dir;
	std::string format;

	float height;	///< Camera height
	float pitch;	///< Camera pitch

	cv::Mat ICurr;	///< Current iteration image
	cv::Mat ILast;	///< Last iteration image
	cv::Rect roi;	///< ROI used to get the current image

	cv::Mat map;	///< Matrix where the map will be stored

	/*
	 * Default values for the ROI position and size.
	 * The ROI is used as the limiter for the error map calculation.
	 */
	int POSX; ///< X coordinate of the upper-left corner of the ROI
	int POSY; ///< Y coordinate of the upper-left corner of the ROI
	int SIZX; ///< ROI width
	int SIZY; ///< ROI Height

	try
	{

		/* ******************************
		 * TCLAP:
		 *
		 * Parsing of the input arguments
		 * ******************************/

		/*
		 * Initializes the parser with a message explaining
		 * the command, a separator for the arguments and
		 * the program's version.
		 */
		TCLAP::CmdLine cmd(
				"Generates the error maps of the distance between two images",
				' ', "1.1");

		/*
		 * dirArg
		 *
		 * The directory argument gets the image sequence
		 * directory.
		 */
		TCLAP::ValueArg<std::string> dirArg("d", "dir",
				"Image sequence directory", true, "", "string");
		cmd.add(dirArg);

		/*
		 * formatArg
		 *
		 * The format argument defines the format to read the
		 * images name and the extension of the images.
		 */
		TCLAP::ValueArg<std::string> formatArg("f", "format",
				"Images sequence format and image extension type", false,
				"%6d.png", "string");
		cmd.add(formatArg);

		/*
		 * sizeArg
		 *
		 * This argument gets the window position and size of
		 * the ROI used for the error map calculation.
		 * The ROI is defined on the first image.
		 */
		TCLAP::ValueArg<std::string> sizeArg("s", "size",
				"Coordinates of the upper-left corner of the ROI followed "
						"by the width and height. The parameters should be separated"
						"by commas (x,y,width,height)", false, "0,0,240,320",
				"string");
		cmd.add(sizeArg);

		/*
		 * sizeArg
		 *
		 * This argument gets the window position and size of
		 * the ROI used for the error map calculation.
		 * The ROI is defined on the first image.
		 */
		TCLAP::ValueArg<std::string> sizeArg("s", "size",
				"Coordinates of the upper-left corner of the ROI followed "
						"by the width and height. The parameters should be separated"
						"by commas (x,y,width,height)", false, "0,0,240,320",
				"string");
		cmd.add(sizeArg);

		// Parsing the args
		cmd.parse(argc, argv);

		/*
		 * Gets the directory from the argument and checks
		 * if it has a valid ending.
		 * Directories should end with a /.
		 */
		dir = dirArg.getValue();
		if (dir[dir.size()] != '/')
			dir = dir + "/";

		/*
		 * Gets the format from the argument.
		 */
		format = formatArg.getValue();

		/*
		 * Get the size from the argument
		 * Parse the argument into POSX, POSY, SIZX, SIZY
		 */
		sscanf(sizeArg.getValue().c_str(), "%d,%d,%d,%d", &POSX, &POSY, &SIZX,
				&SIZY);

		/*
		 * @todo: Create a test with Cucumber for the ROI
		 * size argument validation
		 * 	> Test1: Non null values
		 * 	> Test2: ROI is inside the image
		 */
		fprintf(stdout, "Upper-left corner (%d,%d), Width %d, Height %d\n",
				POSX, POSY, SIZX, SIZY);

		// ROI of the last image
		roi = new cv::Rect(POSX, POSY, SIZX, SIZY);

	} catch (TCLAP::ArgException &e)
	{
		/*
		 * Argument exception handler
		 */

		fprintf(stderr, "Error: %s for arg %s\n", e.error().c_str(),
				e.argId().c_str());
	}

	/*
	 * OpenCV:
	 *
	 * If the arguments all worked
	 * Starts the OpenCV job
	 */
	try
	{
		/*
		 * Open the image sequence as a video stream via
		 * OpenCV library.
		 */
		cv::VideoCapture seq(dir + format);
		for (int i = 0; seq.read(ICurr); i++)
		{
			/*
			 * Tests the image if it's empty.
			 */
			if (ICurr.empty())
			{
				fprintf(stderr, "Failed to open image!\n");
				continue;
			}

			/*
			 * Generates the error map
			 */
			//process_map(ICurr, ILast, roi, map);

			/*
			 * Last image pointing to current image as there
			 * will be a new current image on the next iteration
			 */
			ILast = ICurr;

			/*
			 * Display the images
			 * Breaks if the key 'q' is pressed
			 */
			cv::imshow(WINDOW_NAME, ICurr);
			char key = cv::waitKey(10);
			if (key == 'q')
				break;
		}
	} catch (cv::Exception &e)
	{
		/*
		 * OpenCV exception handle
		 */
		fprintf(stderr, "Error: %s on function %s on line %d\n", e.err.c_str(),
				e.func.c_str(), e.line);
	}

	printf("Finished run!\n");

	return 0;
}

void process_map(cv::Mat curr, cv::Mat last, cv::Mat H, cv::Mat &map)
{

}

void change_reference(cv::Mat Ia, cv::Mat &Ib, cv::Mat aHb)
{

}
