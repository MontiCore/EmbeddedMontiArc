#include "ba_ballTracking.h"
#include "ConvHelper.h"
#include <opencv2/highgui/highgui.hpp>
#include <armadillo>

using namespace arma;
using std::string;

int main(int argc, const char** argv) {

    
    string filename = "BallTrack.mp4";
    cv::VideoCapture capture(filename);
    cv::Mat frame;

    if (!capture.isOpened()) {
        throw "Error when trying to read BallTrack.mp4";
    }

    ba_ballTracking ballT_Object;
    ballT_Object.init();
    //GaussianBlur
    ballT_Object.sizeXIn = 11;
    ballT_Object.sizeYIn = 11;
    ballT_Object.sigmaXIn = 0;
    ballT_Object.sigmaYIn = 0;
    //CvtColor
    ballT_Object.colorConversionIn = 40;
    //InRange
    ballT_Object.lowerBoundaryIn = { 20, 100, 100 };
    ballT_Object.upperBoundaryIn = { 30, 255, 255 };
    //Erode
    ballT_Object.erosion_elemIn = 0;
    ballT_Object.iterationsIn1 = 2;
    //Dilate
    ballT_Object.dilation_elemIn = 0;
    ballT_Object.iterationsIn2 = 2;
    //FindContours
    ballT_Object.modeIn = 0;
    ballT_Object.methodIn = 2;
    //rectangle
    ballT_Object.colorIn = { 0,0,0 };
    ballT_Object.thicknessIn = 4;
    ballT_Object.lineTypeIn = 4;
    

    cv::namedWindow("w", 1);

    for ( ; ; ) {
        capture >> frame;
        if (frame.empty())
            break;
        ballT_Object.imageIn = to_armaCube<unsigned char, 3>(frame);
        ballT_Object.execute();
        cv::Mat output_frame = to_cvmat<unsigned char>(ballT_Object.modImageOut);

    //    cv::flip(output_frame, output_frame, 0);
        //video.write(output_frame);
        cv::imshow("w", output_frame);
        cv::waitKey(30);
    }
    cv::waitKey(0);
}
