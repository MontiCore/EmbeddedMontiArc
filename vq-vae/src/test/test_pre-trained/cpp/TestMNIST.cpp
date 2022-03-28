/* (c) https://github.com/MontiCore/monticore */
#include "CNNTranslator.h"
#include "mnist_mnistClassifier.h"
#include "mnist_encoder.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <armadillo>
#include <string> //Check if it can be removed
#include <iostream>
#include <map>

int main(int argc, char* argv[]) {
	std::cout << argc << std::endl;

	std::vector<std::string> classNames = {"0","1","2","3","4","5","6","7","8","9"};


    if(argc < 2){
        std::cout << "Missing argument: Path to test image must be provided " << std::endl;
        exit(1);
    }
    std::string filePath = argv[1];

    if (!std::ifstream(filePath).good()) {
        std::cerr << "Image loading failure, test image '" << filePath << "' does not exist." << std::endl;
        exit(1);
    }

    cv::Mat img = cv::imread(filePath, cv::IMREAD_GRAYSCALE);
    std::cout << "== original image size: " << img.size() << " ==" << std::endl;

    // scale image to fit
    cv::Size scale(28,28);
    cv::resize(img, img, scale);
    std::cout << "== simply resize: " << img.size() << " ==" << std::endl;


    size_t channels = 1;
    size_t height = img.rows;
    size_t width = img.cols;
    vector<float> data(channels*height*width);

    for(size_t j=0; j<width; j++){
        for(size_t k=0; k<height; k++){
            cv::Vec3b intensity = img.at<cv::Vec3b>(j, k);
            for(size_t i=0; i<channels; i++){
                data[i*height*width + j*height + k] = ((float)intensity[i])/255.0;
            }
        }
    }

    // Encode Images
    mnist_encoder encoder;
    encoder.init();

    encoder.data = CNNTranslator::translateToCube(data, vector<size_t> {channels,height,width} );

    encoder.execute();

    //latent code maps
    arma::cube img_latent_cube = ((1-encoder.encoding)+1)*125.5;
	vector<float> t_img_vec = CNNTranslator::translate(img_latent_cube);
	cv::Mat l_img_cv = cv::Mat(t_img_vec, false);
	l_img_cv = l_img_cv.reshape(1, 7);
	cv::resize(l_img_cv, l_img_cv, cv::Size(), 16, 16, cv::INTER_LINEAR);
	l_img_cv.convertTo(l_img_cv, CV_8UC1);
	cv::imwrite("code_" + std::to_string(std::stoi(argv[2])) + ".png", l_img_cv);

    // Reconstruct and Classify
    mnist_mnistClassifier classifier;
    classifier.init();

	classifier.data = encoder.encoding;

    classifier.execute();

    int classIndex = (int)classifier.classIndex;
    std::string className = "";
    if (classIndex < classNames.size()){
        className = classNames[classIndex];
    }
    double classProbability = classifier.probability * 100;
    int expectedClass = std::stoi(argv[2]);

    // Save image for Debugging Test
    arma::cube img_result_cube = ((1-classifier.decoder.data*3) + 1) * 127.5; // *3 is just a constant to highlight the digits more clearly
	vector<float> img_vec = CNNTranslator::translate(img_result_cube);
	cv::Mat result_img_cv = cv::Mat(img_vec, false);
	result_img_cv = result_img_cv.reshape(1, 28);
	cv::resize(result_img_cv, result_img_cv, cv::Size(), 16, 16, cv::INTER_LINEAR);
	result_img_cv.convertTo(result_img_cv, CV_8UC1);
	cv::imwrite("generated_digit_expected_" + std::to_string(expectedClass) + "_predicted_"+ className +".png", result_img_cv);
	//cv::namedWindow("MNIST");
	//cv::imshow("MNIST", result_img_cv);
	//cv::waitKey(0);
	//cv::destroyWindow("MNIST");

    try {
        if (classIndex == expectedClass) {
            std::cout << "== Predicted expected class for generated image: " << className << " == with probability: " << classProbability << "% ==" << std::endl;
        } else {
            throw (classIndex);
        }
    }
    catch (int falsePrediction) {
        std::cerr << "Test failed" << std::endl;
        std::cerr << "Expected digit: " << expectedClass << std::endl;
        std::cerr << "Predicted: " << falsePrediction << std::endl;
        return 1;
    }

    return 0;
}

