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
mat hierarchy; //Try without hierarchy
mat contours;
void init()
{
image=mat(n,m);
hierarchy=mat(x,y);
contours=mat(w,z);
}
void execute()
{
findContours(image, contours, hierarchy, mode, method);
}

};
#endif