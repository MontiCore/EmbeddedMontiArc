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
mat src;
int sizeX;
int sizeY;
double sigmaX;
double sigmaY;
mat dst;
void init()
{
src=mat(n,m);
dst=mat(n,m);
}
void gaussianBlurHelper(mat src, mat dst, int sizeX, int sizeY, double sigmaX, double sigmaY)
{
    gaussianBlur(src, dst, Size(sizeX, sizeY), sigmaX, sigmaY);
}
void execute()
{
gaussianBlurHelper(src, dst, sizeX, sizeY, sigmaX, sigmaY);
}

};
#endif
