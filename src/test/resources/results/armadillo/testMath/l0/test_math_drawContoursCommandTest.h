#ifndef TEST_MATH_DRAWCONTOURSCOMMANDTEST
#define TEST_MATH_DRAWCONTOURSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace cv;
class test_math_drawContoursCommandTest{
public:
void init()
{
}
void execute()
{
double a = (drawContours(0, 0, 0, 0, 0));
}

};
#endif
