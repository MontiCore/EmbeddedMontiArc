#ifndef TEST_MATH_INRANGECOMMANDTEST
#define TEST_MATH_INRANGECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/core.hpp"
using namespace arma;
using namespace cv;
class test_math_inRangeCommandTest{
public:
void init()
{
}
void execute()
{
double a = (inRange(0, 0, 0, 0, 0));
}

};
#endif
