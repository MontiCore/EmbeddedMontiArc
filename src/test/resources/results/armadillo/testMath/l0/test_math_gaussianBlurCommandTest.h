#ifndef TEST_MATH_GAUSSIANBLURCOMMANDTEST
#define TEST_MATH_GAUSSIANBLURCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace cv;
class test_math_gaussianBlurCommandTest{
public:
cube src;
int sizeX;
int sizeY;
double sigmaX;
double sigmaY;
cube dst;
void init()
{
src = cube(3, n, m);
dst = cube(3, n, m);
}
void gaussianBlurHelper(cube src, cube dst, int sizeX, int sizeY, double sigmaX, double sigmaY)
{
    gaussianBlur(src, dst, Size(sizeX, sizeY), sigmaX, sigmaY);
}
void execute()
{
gaussianBlurHelper(src, dst, sizeX, sizeY, sigmaX, sigmaY);
}

};
#endif
