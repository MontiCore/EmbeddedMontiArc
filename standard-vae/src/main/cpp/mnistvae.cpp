/* (c) https://github.com/MontiCore/monticore */
#include "CNNTranslator.h"
#include "vae_connector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <armadillo>
#include <string> //Check if it can be removed
#include <iostream>
#include <map>

int main(int argc, char* argv[]) {
	std::cout << argc << std::endl;

	// Code Sampling from normal distribution
	//int genSeed = std::stoi(argv[1]);
	//srand(genSeed);
	//std::default_random_engine generator;
	//generator.seed(genSeed);
	//std::normal_distribution<double> distribution(0.0,1.0);

    vector<float> data(2);

	//for(size_t i=0; i < 2; i++){
	//	data[i] = distribution(generator);
	//}
	data[0] = std::stod(argv[1]);
	data[1] = std::stod(argv[2]);

    // EMADL generated VAE model -------------------------------------------------
    bvae_connector connector;
    connector.init();

	connector.data = conv_to< arma::Col<double> >::from( CNNTranslator::translateToCol(data,
				vector<size_t> {2}) );

    connector.execute();

	arma::cube img_result_cube = ((1-connector.res*3) + 1) * 127.5; //*3 to highlight the digit
	// ----------------------------------------------------------------------------

    // Save image
	vector<float> img_vec = CNNTranslator::translate(img_result_cube);
	cv::Mat result_img_cv = cv::Mat(img_vec, false);
	result_img_cv = result_img_cv.reshape(1, 28);
	cv::resize(result_img_cv, result_img_cv, cv::Size(), 16, 16, cv::INTER_LINEAR);
	result_img_cv.convertTo(result_img_cv, CV_8UC1);
	cv::imwrite("generated_digit.png", result_img_cv);
	cv::namedWindow("MNIST");
	cv::imshow("MNIST", result_img_cv);
	cv::waitKey(0);
	cv::destroyWindow("MNIST");

    return 0;
}

