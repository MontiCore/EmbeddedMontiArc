#ifndef TEST_MATH_FINDCONTOURSCOMMANDTEST
#define TEST_MATH_FINDCONTOURSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace cv;
class test_math_findContoursCommandTest{
public:
void init()
{
}
void execute()
{
double a = (findContours(0, 0, 0, 0, 0));
}

};
#endif
