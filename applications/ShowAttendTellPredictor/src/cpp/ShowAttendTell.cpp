#include "CNNTranslator.h"
#include "showAttendTell_main.h"

#include <fstream>
#include <iostream>
#include <map>
#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const int MAX_LENGTH = 20;

std::vector<std::string> readDict(const std::string& filename) {

    std::ifstream file(filename);
    if (!file) {
        throw std::runtime_error("File " + filename + " not found");
    }

    int i = 0;
    std::string buffer;
    std::vector<std::string> vector;

    while (std::getline(file, buffer)) {
        vector.push_back(buffer);
    }

    return vector;
}



std::string* indicesToSequence(const std::vector<int>& indices, const std::vector<std::string>& vocab) {
    static std::string sequence[MAX_LENGTH];

   for(std::size_t i=0; i<indices.size(); ++i)  {
        if (indices[i] < vocab.size()) {
            sequence[i] = (vocab.at(indices[i]));
        } else {
            throw std::runtime_error("Invalid target vocab size");
        }
    }

    return sequence;
}



int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: ShowAttendTell <image path> <path to dict.txt>" << std::endl;
        return 1;
    }

    try {

		// read image
		std::string path = argv[1];
		cv::Mat img = cv::imread(path);
		std::cout << "### original image size: " << img.size() << " ###" << std::endl;

		// scale image
		cv::Size scale(224,224);
		cv::resize(img, img, scale);
		std::cout << "### resized image size:  " << img.size() << " ###" << std::endl;

		// save image to data
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

		// initialize network
        showAttendTell_main showAttendTell;
        showAttendTell.init();

        showAttendTell.images = conv_to<icube>::from(CNNTranslator::translateToCube(data, vector<size_t> {channels,height,width}));

		// run network
        showAttendTell.execute();

		// get results from network
        std::vector<int> targetIndices;
        for (std::size_t i = 0; i < MAX_LENGTH; ++i) {
            targetIndices.push_back(showAttendTell.target[i](0));
        }

		// transform result indices to words
        std::string *targetSequence;
		std::vector<std::string> dict = readDict(argv[2]);
		targetSequence = indicesToSequence(targetIndices, dict);

		// print result sentence
		for (std::size_t i = 0; i < MAX_LENGTH; ++i) {
        	std::cout << targetSequence[i] << std::endl;
		}

    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
