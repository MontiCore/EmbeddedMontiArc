#ifndef TESTA_CUBE_IMAGEMATCHER_H
#define TESTA_CUBE_IMAGEMATCHER_H

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;
using std::cout;
using std::endl;

class ImageMatcher{
public:
    static int drawMatch( const cv::Mat& img1, const cv::Mat& img2, const String& matchedImage )
    {
        if ( img1.empty() || img2.empty() )
        {
            cout << "Could not open or find the image!\n" << endl;
            return -1;
        }
        cv::Mat img_matches;
        int rows = img1.rows;
        int cols = img1.cols;
        cout << "img1 rows:" << rows << endl;
        cout << "img1 cols:" << cols << endl;
        rows = img2.rows;
        cols = img2.cols;
        cout << "img2 rows:" << rows << endl;
        cout << "img2 cols:" << cols << endl;
        cv::absdiff(img1, img2, img_matches);
        cv::bitwise_not(img_matches, img_matches);
        cv::imwrite(matchedImage, img_matches);
        return 0;
    }
};



#endif //TESTA_CUBE_IMAGEMATCHER_H
