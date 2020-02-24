#ifndef TEST_MATH_DILATECOMMANDTEST
#define TEST_MATH_DILATECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
#include "ConvHelper.h"
using namespace arma;
using namespace std;
class test_math_dilateCommandTest{
public:
arma::mat src;
int dilation_elem;
int iterations;
arma::mat dst;
void init()
{
src=mat(n,m);
dst=mat(n,m);
}
void dilateHelper(arma::mat src, arma::mat dst, int dilation_elem, int iterations)
{
    int dilation_type = 0;
    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
    dilation_size = dilation_elem;
    cv::mat element = cv::getStructuringElement( dilation_type,
                            Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                            Point( -1, -1 ) );
    cv::Mat srcCV;
    cv::Mat dstCV;
    srcCV = ConvHelper::to_cvmat(src);
    cv::dilate( srcCV, dstCV, element, Point(-1,-1), iterations );
    dst = ConvHelper::to_arma(dstCV);
}
void execute()
{
dilateHelper(src, dst, dilation_elem, iterations);
}

};
#endif
