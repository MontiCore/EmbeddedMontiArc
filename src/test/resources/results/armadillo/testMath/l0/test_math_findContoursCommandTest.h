#ifndef TEST_MATH_FINDCONTOURSCOMMANDTEST
#define TEST_MATH_FINDCONTOURSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
#include "ConvHelper.h"
#include <vector>
using namespace arma;
using namespace std;
class test_math_findContoursCommandTest{
public:
arma::mat image;
int mode;
int method;
vector<vector<cv::Point>> contours;
void init()
{
image=mat(n,m);
}
void findContoursHelper(arma::mat image, vector<vector<cv::Point>> contours, int mode, int method)
{
    cv::Mat srcCV;
    srcCV = ConvHelper::to_cvmat(src);
    cv::findContours( image, contours, mode, method );
}
void execute()
{
cv::findContours(image, contours, mode, method);
}

};
#endif