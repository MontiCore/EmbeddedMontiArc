#ifndef TEST_MATH_ERODEANDGAUSSIANBLURCOMMANDTEST
#define TEST_MATH_ERODEANDGAUSSIANBLURCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "test_math_erodeAndGaussianBlurCommandTest_gaussBlur.h"
#include "test_math_erodeAndGaussianBlurCommandTest_arm.h"
#include "test_math_erodeAndGaussianBlurCommandTest_cvtColor.h"
using namespace arma;
class test_math_erodeAndGaussianBlurCommandTest{
public:
Cube<unsigned char> imageIn;
int sizeXIn;
int sizeYIn;
double sigmaXIn;
double sigmaYIn;
int colorConversionIn;
Cube<unsigned char> modImageOut;
test_math_erodeAndGaussianBlurCommandTest_gaussBlur gaussBlur;
test_math_erodeAndGaussianBlurCommandTest_arm arm;
test_math_erodeAndGaussianBlurCommandTest_cvtColor cvtColor;
void init()
{
imageIn = Cube<unsigned char>(960, 720, 3);
modImageOut = Cube<unsigned char>(960, 720, 3);
gaussBlur.init();
arm.init();
cvtColor.init();
}
void execute()
{
gaussBlur.src = imageIn;
gaussBlur.sizeX = sizeXIn;
gaussBlur.sizeY = sizeYIn;
gaussBlur.sigmaX = sigmaXIn;
gaussBlur.sigmaY = sigmaYIn;
gaussBlur.execute();
arm.src = gaussBlur.dst;
arm.execute();
cvtColor.src = arm.dst;
cvtColor.colorConversion = colorConversionIn;
cvtColor.execute();
modImageOut = cvtColor.dst;
}

};
#endif
