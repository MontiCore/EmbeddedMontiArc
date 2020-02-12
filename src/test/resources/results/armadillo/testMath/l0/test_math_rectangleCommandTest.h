#ifndef TEST_MATH_RECTANGLECOMMANDTEST
#define TEST_MATH_RECTANGLECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace cv;
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
Mat rectangleHelper(Mat src, Rect rect, colvec color, int thickness, int lineType)
{
    cv::rectangle(src, rect.tl(), rect.br(), Scalar(color(0), color(1), color(2)), thickness, lineType);
    return src;
}
void execute()
{
outputImg = (rectangleHelper(src, rect, color, thickness, lineType));
}

};
#endif
