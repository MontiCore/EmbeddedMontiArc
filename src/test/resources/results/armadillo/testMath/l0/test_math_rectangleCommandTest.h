#ifndef TEST_MATH_RECTANGLECOMMANDTEST
#define TEST_MATH_RECTANGLECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "ConvHelper.h"
#include <opencv2/imgproc/imgproc.hpp>
using namespace arma;
using namespace std;
class test_math_rectangleCommandTest{
public:
Cube<unsigned char> src;
cv::Rect rect;
colvec color;
int thickness;
int lineType;
Cube<unsigned char> outputImg;
void init()
{
src = Cube<unsigned char>(3, n, m);
color=colvec(3);
outputImg = Cube<unsigned char>(3, n, m);
}
Cube<unsigned char> rectangleHelper(const Cube<unsigned char>& src, cv::Rect& rect, colvec color, int thickness, int lineType)
{
    cv::Mat srcCV;
    srcCV = to_cvmat<unsigned char>(src);
    cv::rectangle(srcCV, rect.tl(), rect.br(), cv::Scalar(color(0), color(1), color(2)), thickness, lineType);
    arma::Cube<unsigned char> srcCube;
    srcCube = to_armaCube<unsigned char, 3>(srcCV);
    return srcCube;
}
void execute()
{
outputImg = (rectangleHelper(src, rect, color, thickness, lineType));
}

};
#endif