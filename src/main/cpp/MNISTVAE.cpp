/* (c) https://github.com/MontiCore/monticore */
#include <CNNTranslator.h>
#include "mnistvae_connector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <armadillo>
#include <string> //Check if it can be removed
#include <iostream>
#include <stdlib.h>  
#include <map>
#include <random>

int main(int argc, char* argv[]) {
    
	//std::cout << argc << std::endl;

    mnistvae_connector connector;
    connector.init();

	int genSeed = std::stoi(argv[1]);
	srand(genSeed);
	std::default_random_engine generator;
	generator.seed(genSeed);
	std::normal_distribution<double> distribution(0.0,1.0);

    colvec data(2);

	for(size_t i=0; i < 2; i++){
		data[i] = distribution(generator);
	}

	connector.data = data;

    connector.execute();

    arma::cube img_result_cube = (connector.res + 1) * 127.5;

    vector<float> img_vec = CNNTranslator::translate(img_result_cube);

    cv::Mat result_img_cv = cv::Mat(img_vec, false);
    result_img_cv = result_img_cv.reshape(1, 28);
    cv::resize(result_img_cv, result_img_cv, cv::Size(), 16, 16, cv::INTER_LINEAR);
    result_img_cv.convertTo(result_img_cv, CV_8UC3);

    cv::namedWindow("MNIST");
    cv::imshow("MNIST", result_img_cv);
    cv::waitKey(0);
    cv::destroyWindow("MNIST");



    return 0;
}
