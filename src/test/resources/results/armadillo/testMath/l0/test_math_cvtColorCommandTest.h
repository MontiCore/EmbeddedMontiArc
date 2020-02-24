#ifndef TEST_MATH_CVTCOLORCOMMANDTEST
#define TEST_MATH_CVTCOLORCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
#include "ConvHelper.h"
using namespace arma;
using namespace std;
class test_math_cvtColorCommandTest{
public:
arma::mat src;
int colorConversion;
arma::mat dst;
void init()
{
src=mat(n,m);
dst=mat(n,m);
}
void cvtColorHelper(arma::mat src, arma::mat dst, int colorConversion)
{
    cv::Mat srcCV;
    cv::Mat dstCV;
    srcCV = ConvHelper::to_cvmat(src);
    cv::cvtColor(srcCV, dstCV, colorConversion);
    dst = ConvHelper::to_arma(dstCV);
}
void execute()
{
cvtColorHelper(src, dst, colorConversion);
}

};
#endif
