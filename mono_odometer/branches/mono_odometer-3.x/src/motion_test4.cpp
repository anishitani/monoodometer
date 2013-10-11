/*
 * motion_teste.cpp
 *
 *  Created on: Dec 16, 2012
 *      Author: nishitani
 * Description: Teste da estimação da matriz F/E e teste
 * 				da decomposição de E em R|t.
 */

#include "core.h"
#include "image_processor.cpp"
#include "motion_processor.cpp"

int read_param(FILE* file, char* img_name, double *k, double *gt)
{
	int err;
	err =
			fscanf(file,
					"%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
					img_name, &k[0], &k[1], &k[2], &k[3], &k[4], &k[5], &k[6],
					&k[7], &k[8], &gt[0], &gt[1], &gt[2], &gt[4], &gt[5],
					&gt[6], &gt[8], &gt[9], &gt[10], &gt[3], &gt[7], &gt[11]);
	gt[12] = gt[13] = gt[14] = gt[15] = 0;
	return err;
}

/**
 *	main()
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "motion_test4");
	std::string node_name = ros::this_node::getName();

	ros::NodeHandle nh(node_name);

	LRM::ImageProcessor img_proc;
	LRM::MotionProcessor mot_proc;
	LRM::MotionProcessorParameter mot_proc_parameter;
	LRM::ImageProcessorParameter img_proc_parameter;
	img_proc_parameter.parse(nh);
	mot_proc_parameter.parse(nh);

	img_proc_parameter.setParameterByName("FEATURE_TYPE", LRM::FAST);
	mot_proc_parameter.setParameterByName("RANSAC_EPIPOLAR_DIST", 0.1);

	img_proc.setting(img_proc_parameter);
	mot_proc.setting(mot_proc_parameter);

	cv::Mat train_img, query_img;

	std::vector<cv::Point2d> train_pts, query_pts;
	std::vector<cv::KeyPoint> train_kpts, query_kpts;
	std::vector<cv::DMatch> matches;

	double gt[4 * 4]; 	// ground truth
	double k[9];	 	// camera matrix
	cv::Mat K, GT, pose, motion;

	std::FILE *param_file;
	char img_name[100];

	param_file =
			fopen(
					"/home/nishitani/Dropbox/Photos/Mestrado/dataset/other/dino/dino_par.txt",
					"r");

	int n;
	int err = fscanf(param_file, "%d", &n);

	char c = 0;
	bool first_run;

	cv::namedWindow("opt_flow");

	for (int i = 0; i < n && c != 'q'; i++)
	{
		//le parametros do arquivo
		read_param(param_file, img_name, k, gt);

		//le imagem
		char img_path[100];
		sprintf(img_path,
				"/home/nishitani/Dropbox/Photos/Mestrado/dataset/other/dino/%04d.png",
				i + 1);
		query_img = cv::imread(img_path, CV_LOAD_IMAGE_GRAYSCALE);

		/*
		 * Detecta as feature e realiza o match utilizando
		 * optical flow como referencia para match
		 */
		img_proc.detect_features(query_img, query_kpts);
		if (train_kpts.empty())
		{
			K = cv::Mat(3, 3, CV_64F, k);	// define a matriz de calibracaao
			GT = cv::Mat(4, 4, CV_64F, gt);	// define a pose da camera segundo o ground truth
			GT.copyTo(pose);
			first_run = true;
		}
		else
		{
			GT = cv::Mat(4, 4, CV_64F, gt); // atualiza o ground truth
			img_proc.match_features_optflow(query_img, train_img, query_kpts,
					train_kpts, matches);
			mot_proc.matches2points(train_kpts, query_kpts, matches, train_pts,
					query_pts);
			first_run = false;
		}

		// estimacao do movimento
		if (!first_run)
		{
			if (matches.size() > 8) // mais de 8 correspondencia
			{
				mot_proc.estimate_motion(train_pts, query_pts, matches, K);
			}
			else // com menos de 8 correspondencia nao ha movimento
			{
				mot_proc.noMotion();
			}
			motion = mot_proc.getCameraMatrix().inv();
			pose = motion * pose;

			std::cout << "Motion: \n" << motion << std::endl << std::endl;
		}

		std::cout << "Ground Truth: \n" << GT << std::endl << std::endl;
		std::cout << "My Pose: \n" << pose << std::endl << std::endl;

		// display imagem
		cv::Mat out;
		img_proc.draw_displacement(query_img, out, query_kpts, train_kpts,
				matches, mot_proc.getInlierMask());
		cv::imshow("opt_flow", out);

		train_kpts.clear();
		train_kpts = query_kpts;
		train_img = query_img;

		c = cv::waitKey();
	}
	fclose(param_file);

	return 0;
}
