#ifndef TEST_MATH_RECTANGLECOMMANDTEST
#define TEST_MATH_RECTANGLECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
#include "ConvHelper.h"
using namespace arma;
using namespace std;
class test_math_rectangleCommandTest{
public:
cube src;
Rect rect;
colvec color;
int thickness;
int lineType;
cube outputImg;
void init()
{
src = cube(3, n, m);
color=colvec(3);
outputImg = cube(3, n, m);
}
cube rectangleHelper(cube src, cv::Rect rect, colvec color, int thickness, int lineType)
{
    cv::Mat srcCV;
    srcCV = ConvHelper::to_cvmat(src);
    cv::rectangle(srcCV, rect.tl(), rect.br(), Scalar(color(0), color(1), color(2)), thickness, lineType);
    arma::cube srcCube;
    srcCube = to_armaCube(srcCV);
    return srcCube;
}
void execute()
{
outputImg = (rectangleHelper(src, rect, color, thickness, lineType));
}

};
#endif

