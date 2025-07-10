#ifndef TEST_MATH_GAUSSIANBLURCOMMANDTEST
#define TEST_MATH_GAUSSIANBLURCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "ConvHelper.h"
#include <opencv2/imgproc/imgproc.hpp>
using namespace arma;
using namespace std;
class test_math_gaussianBlurCommandTest{
public:
Cube<unsigned char> src;
int sizeX;
int sizeY;
double sigmaX;
double sigmaY;
Cube<unsigned char> dst;
void init()
{
src = Cube<unsigned char>(n, m, 3);
dst = Cube<unsigned char>(n, m, 3);
}
void gaussianBlurHelper(const Cube<unsigned char>& src, Cube<unsigned char>& dst, int sizeX, int sizeY, double sigmaX, double sigmaY)
{
    cv::Mat srcCV;
    cv::Mat dstCV;
    srcCV = to_cvmat<unsigned char>(src);
    cv::Size sizeO = cv::Size(sizeX, sizeY);
    cv::GaussianBlur(srcCV, dstCV, sizeO, sigmaX, sigmaY);
    dst = to_armaCube<unsigned char, 3>(dstCV);
}
void execute()
{
gaussianBlurHelper(src, dst, sizeX, sizeY, sigmaX, sigmaY);
}

};
#endif