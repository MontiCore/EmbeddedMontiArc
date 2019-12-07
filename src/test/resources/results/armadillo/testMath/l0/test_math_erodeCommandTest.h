#ifndef TEST_MATH_ERODECOMMANDTEST
#define TEST_MATH_ERODECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace cv;
class test_math_erodeCommandTest{
public:
mat arrayIn;
mat arrayOut;
void init()
{
arrayIn=mat(n,m);
arrayOut=mat(n,m);
}
void execute()
{
arrayOut = (erode(arrayIn));
}

};
#endif
