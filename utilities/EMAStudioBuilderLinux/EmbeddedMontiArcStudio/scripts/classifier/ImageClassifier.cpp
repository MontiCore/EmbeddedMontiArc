/* (c) https://github.com/MontiCore/monticore */
#include <armadillo>
#include <iomanip>
#include <iostream>
#include "CNNTranslator.h"
#include "cifar10_main.h"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[]){

    const std::vector<std::string> classNames {"airplane","automobile","bird","cat","deer","dog","frog","horse","ship","truck"};

    if(argc < 2){
        std::cout << "Missing Argument: An argument has to denote the path to the tested image " << std::endl;
        exit(1);
    }
    std::string filePath = argv[1];

    ifstream file(filePath);
    if (file.is_open()){
        file.close();
    }
    else{
        std::cout << "File '" << filePath << "' does not exist or couldn't be opened" << std::endl;
        exit(1);
    }

    cv::Mat img = cv::imread(filePath);

    size_t channels = 3;
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



    cifar10_main classifier;
    classifier.init();
    classifier.image = CNNTranslator::translateToCube(data, vector<size_t> {channels,height,width});
    classifier.execute();

    int classIndex = (int)classifier.classIndex;
    std::string className = "";
    if (classIndex < classNames.size()){
        className = classNames[classIndex];
    }
    std::cout << std::endl;
    std::cout << "Predicted class for image " << filePath << ": " << className << " (" << std::to_string(classIndex) << ")" << std::endl;

    std::ofstream outputFile("class.txt");
    outputFile << className << " (" << std::to_string(classIndex) << ")";
    outputFile.close();
    return 0;
}
