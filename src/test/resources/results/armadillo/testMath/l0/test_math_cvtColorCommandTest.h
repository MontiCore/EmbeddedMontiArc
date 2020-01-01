#ifndef TEST_MATH_CVTCOLORCOMMANDTEST
#define TEST_MATH_CVTCOLORCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace cv;
class test_math_cvtColorCommandTest{
public:
mat src;
int colorConversion;
mat dst;
void init()
{
src=mat(n,m);
dst=mat(n,m);
}
void execute()
{
cvtColor(src, dst, colorConversion);
}

};
#endif