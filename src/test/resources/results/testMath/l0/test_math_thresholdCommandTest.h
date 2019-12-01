#ifndef TEST_MATH_THRESHOLDCOMMANDTEST
#define TEST_MATH_THRESHOLDCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace cv;
class test_math_thresholdCommandTest{
public:
void init()
{
}
void execute()
{
double a = (threshold(0, 0, 0, 0, 0));
}

};
#endif
