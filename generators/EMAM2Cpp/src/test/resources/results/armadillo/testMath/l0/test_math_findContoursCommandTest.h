#ifndef TEST_MATH_FINDCONTOURSCOMMANDTEST
#define TEST_MATH_FINDCONTOURSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "ConvHelper.h"
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
using namespace arma;
using namespace std;
class test_math_findContoursCommandTest{
public:
arma::Mat<unsigned char> image;
int mode;
int method;
std::vector<std::vector<cv::Point>> contours;
void init()
{
image=arma::Mat<unsigned char>(n,m);
}
void findContoursHelper(const arma::Mat<unsigned char>& image, std::vector<std::vector<cv::Point>>& contours, int mode, int method)
{
    cv::Mat srcCV;
    srcCV = to_cvmat<unsigned char>(image);
    cv::findContours( srcCV, contours, mode, method );
}
void execute()
{
findContoursHelper(image, contours, mode, method);
}

};
#endif
