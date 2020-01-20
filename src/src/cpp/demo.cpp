/* (c) https://github.com/MontiCore/monticore */
#include "CNNTranslator.h"
#include "imageSegmentation_connector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <armadillo>
#include <string> //Check if it can be removed
#include <iostream>
#include <map>

int main(int argc, char* argv[]) {
    if(argc < 2){ //Note: argc=1 if no arguments are provided
        std::cout << "Missing argument: Path to 6 test image " << std::endl;
        exit(1);
    }


    cNNCalculator_connector connector;
    connector.init();

    // dar
    std::string filePath = argv[n+1];

    if (!std::ifstream(filePath).good()) {
        std::cerr << "Image loading failure, test image '" << filePath << "' does not exist." << std::endl;
        exit(1);
    }

    cv::Mat img = cv::imread(filePath);
    std::cout << "== original image size: " << img.size() << " ==" << std::endl;

    // scale image to fit
    cv::Size scale(480,480);
    cv::resize(img, img, scale);
    std::cout << "== simply resize: " << img.size() << " ==" << std::endl;

    size_t channels = 1;
    size_t height = img.rows;
    size_t width = img.cols;
    vector<float> data(channels*height*width);

    for(size_t j=0; j<height; j++){
        for(size_t k=0; k<width; k++){
            cv::Vec3b intensity = img.at<cv::Vec3b>(j, k);
            for(size_t i=0; i<channels; i++){
                data[i*height*width + j*height + k] = (float) intensity[i];
            }
        }
    }

    connector.image = conv_to< icube >::from( CNNTranslator::translateToCube(data, vector<size_t> {channels,height,width}) );


    connector.execute();

    int classIndex = (int)connector.res;
    std::cout << "== SUM: " << classIndex << std::endl;

    return 0;
}
