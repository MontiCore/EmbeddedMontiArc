#ifndef TEST_MATH_DILATECOMMANDTEST
#define TEST_MATH_DILATECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "ConvHelper.h"
#include <opencv2/imgproc/imgproc.hpp>
using namespace arma;
using namespace std;
class test_math_dilateCommandTest{
public:
arma::Mat<unsigned char> src;
int dilation_elem;
int iterations;
arma::Mat<unsigned char> dst;
void init()
{
src=arma::Mat<unsigned char>(n,m);
dst=arma::Mat<unsigned char>(n,m);
}
void dilateHelper(const arma::Mat<unsigned char>& src, arma::Mat<unsigned char>& dst, int dilation_elem, int iterations)
{
    int dilation_type = 0;
    if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }
    int dilation_size = dilation_elem;
    cv::Mat element = cv::getStructuringElement( dilation_type,
                            cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                            cv::Point( -1, -1 ) );
    cv::Mat srcCV;
    cv::Mat dstCV;
    srcCV = to_cvmat<unsigned char>(src);
    cv::dilate( srcCV, dstCV, element, cv::Point(-1,-1), iterations );
    dst = to_arma<unsigned char>(dstCV);
}
void execute()
{
dilateHelper(src, dst, dilation_elem, iterations);
}

};
#endif
