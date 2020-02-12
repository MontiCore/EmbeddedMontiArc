#ifndef TEST_MATH_FINDCONTOURSCOMMANDTEST
#define TEST_MATH_FINDCONTOURSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
#include <vector>
using namespace arma;
using namespace cv;
class test_math_findContoursCommandTest{
public:
mat image;
int mode;
int method;
vector<vector<cv::Point>> contours;
void init()
{
image=mat(n,m);
}
void execute()
{
findContours(image, contours, mode, method);
}

};
#endif