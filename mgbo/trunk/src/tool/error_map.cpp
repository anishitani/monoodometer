/*
 * error_map.cpp
 *
 *  Created on: Mar 1, 2014
 *      Author: nishitani
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <tclap/CmdLine.h>

#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
	std::string dir;
	std::string format;

	std::string win("Error Map");

	cv::Mat ICur;

	try
	{

		/* ******************************
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
		 * The format to read the images name and the extension
		 * of the images.
		 */
		TCLAP::ValueArg<std::string> formatArg("f", "format",
				"Images sequence format and image extension type", false,
				"%6d.png", "string");
		cmd.add(formatArg);

		/*
		 * Parsing the args.
		 */
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
		 * Open the image sequence as a video stream via
		 * OpenCV library.
		 */
		cv::VideoCapture seq(dir + format);

		for (int i = 0; seq.read(ICur); i++)
		{
			/*
			 * Tests the image if it's empty.
			 */
			if (ICur.empty())
			{
				fprintf(stderr, "Failed to open image!\n");
				continue;
			}


		}

	} catch (TCLAP::ArgException &e)
	{
		/*
		 * Argument exception handle
		 */
		fprintf(stderr, "Error: %s for arg %s\n", e.error().c_str(),
				e.argId().c_str());
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
