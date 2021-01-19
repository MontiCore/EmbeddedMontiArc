#ifndef TEST_MATH_CVTCOLORCOMMANDTEST
#define TEST_MATH_CVTCOLORCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "ConvHelper.h"
#include <opencv2/imgproc/imgproc.hpp>
using namespace arma;
using namespace std;
class test_math_cvtColorCommandTest{
public:
arma::Mat<unsigned char> src;
int colorConversion;
arma::Mat<unsigned char> dst;
void init()
{
src=arma::Mat<unsigned char>(n,m);
dst=arma::Mat<unsigned char>(n,m);
}
void cvtColorHelper(const arma::Mat<unsigned char>& src, arma::Mat<unsigned char>& dst, int colorConversion)
{
    cv::Mat srcCV;
    cv::Mat dstCV;
    srcCV = to_cvmat<unsigned char>(src);
    cv::cvtColor(srcCV, dstCV, colorConversion);
    dst = to_arma<unsigned char>(dstCV);
}
void execute()
{
cvtColorHelper(src, dst, colorConversion);
}

};
#endif
