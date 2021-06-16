/* (c) https://github.com/MontiCore/monticore */
#include "CNNTranslator.h"
#include "MNISTVAE.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <armadillo>
#include <string> //Check if it can be removed
#include <iostream>
#include <stdlib.h>  
#include <map>
#include <random>

int main(int argc, char* argv[]) {
    
	std::cout << argc << std::endl;

    //mnistvae_connector connector;
    connector.init();

	int genSeed = std::stoi(argv[1]);
	srand(genSeed);
	std::default_random_engine generator;
	generator.seed(genSeed);
	std::normal_distribution<double> distribution(0.0,1.0);

	connector.eps = distribution(generator)

    vector<float> data(100);

	for(size_t i=0; i < 100; i++){
		data[i] = distribution(generator);
	}

	connector.noise = conv_to< arma::Cube<double> >::from( CNNTranslator::translateToCube(data, 
				vector<size_t> {1, 28, 28}) );

    connector.execute();


	//for x in range(0, width -1, GRID_SIZE):
    //	cv::line(img, (x, 0), (x, height), (255, 0, 0), 1, 1)

	//cv::imshow('Hehe', img)
	//cv::waitKey(0)


    return 0;
}
