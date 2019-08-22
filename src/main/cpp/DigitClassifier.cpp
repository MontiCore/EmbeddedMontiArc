/* (c) https://github.com/MontiCore/monticore */
#include "CNNTranslator.h"
#include "mnist_mnistClassifier.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <armadillo>
#include <string>
#include <iostream>
#include <map>

int main(int argc, char** argv) {
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

    cv::Mat img = cv::imread(filePath);
    std::cout << "== original image size: " << img.size() << " ==" << std::endl;

    // scale image to fit
    cv::Size scale(28,28);
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

    mnist_mnistClassifier classifier;
    classifier.init();
    classifier.image = conv_to< icube >::from( CNNTranslator::translateToCube(data, vector<size_t> {channels,height,width}) );
    classifier.execute();

    int classIndex = (int)classifier.classIndex;
    std::string className = "";
    if (classIndex < classNames.size()){
        className = classNames[classIndex];
    }
    double classProbability = classifier.probability * 100;

    std::cout << "== Predicted class for image \"" << filePath << "\": " << className << " == with probability: " << classProbability << "% ==" << std::endl;

    std::ofstream outputFile("class.txt");
    outputFile << className << " (" << std::to_string(classIndex) << ")";
    outputFile.close();
    return 0;
}
