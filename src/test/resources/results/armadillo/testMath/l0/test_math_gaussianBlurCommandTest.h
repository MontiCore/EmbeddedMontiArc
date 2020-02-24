#ifndef TEST_MATH_GAUSSIANBLURCOMMANDTEST
#define TEST_MATH_GAUSSIANBLURCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
#include "ConvHelper.h"
using namespace arma;
using namespace std;
class test_math_gaussianBlurCommandTest{
public:
cube src;
int sizeX;
int sizeY;
double sigmaX;
double sigmaY;
cube dst;
void init()
{
src = cube(3, n, m);
dst = cube(3, n, m);
}
void gaussianBlurHelper(cube src, cube dst, int sizeX, int sizeY, double sigmaX, double sigmaY)
{
    cv::Mat srcCV;
    cv::Mat dstCV;
    srcCV = ConvHelper::to_cvmat(src);
    cv::gaussianBlur(srcCV, dstCV, Size(sizeX, sizeY), sigmaX, sigmaY);
    dst = ConvHelper::to_armaCube(dstCV);
}
void execute()
{
gaussianBlurHelper(src, dst, sizeX, sizeY, sigmaX, sigmaY);
}

};
#endif
