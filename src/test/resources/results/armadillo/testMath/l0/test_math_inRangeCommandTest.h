#ifndef TEST_MATH_INRANGECOMMANDTEST
#define TEST_MATH_INRANGECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "ConvHelper.h"
#include <opencv2/core/core.hpp>
using namespace arma;
using namespace std;
class test_math_inRangeCommandTest{
public:
arma::Mat<unsigned char> src;
colvec lowerBoundary;
colvec upperBoundary;
arma::Mat<unsigned char> dst;
void init()
{
src=arma::Mat<unsigned char>(n,m);
lowerBoundary=colvec(3);
upperBoundary=colvec(3);
dst=arma::Mat<unsigned char>(n,m);
}
void inRangeHelper(const Cube<unsigned char>& src, arma::Mat<unsigned char>& dst, colvec lowerB, colvec upperB)
{
    cv::Mat srcCV;
    cv::Mat dstCV;
    srcCV = to_cvmat<unsigned char>(src);
    cv::inRange(srcCV, cv::Scalar(lowerB(0), lowerB(1), lowerB(2)),
            cv::Scalar(upperB(0), upperB(1), upperB(2)), dstCV);
    dst = to_arma<unsigned char>(dstCV);
}
void execute()
{
inRangeHelper(src, dst, lowerBoundary, upperBoundary);
}

};
#endif