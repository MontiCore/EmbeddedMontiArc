#ifndef TEST_MATH_INRANGECOMMANDTEST
#define TEST_MATH_INRANGECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/core.hpp"
#include "ConvHelper.h"
using namespace arma;
using namespace std;
class test_math_inRangeCommandTest{
public:
mat src;
colvec lowerBoundary;
colvec upperBoundary;
mat dst;
void init()
{
src=mat(n,m);
lowerBoundary=colvec(3);
upperBoundary=colvec(3);
dst=mat(n,m);
}
void inRangeHelper(cube src, mat dst, colvec lowerB, colvec upperB)
{
    cv::inRange(src, Scalar(lowerB(0), lowerB(1), lowerB(2)),
            Scalar(upperB(0), upperB(1), upperB(2)), dst);
}
void execute()
{
inRangeHelper(src, dst, lowerBoundary, upperBoundary);
}

};
#endif
