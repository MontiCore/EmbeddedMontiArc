#ifndef TEST_MATH_ERODECOMMANDTEST
#define TEST_MATH_ERODECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "ConvHelper.h"
#include <opencv2/imgproc/imgproc.hpp>

using namespace arma;
using namespace std;
class test_math_erodeCommandTest{
public:
arma::Mat<unsigned char> src;
int erosion_elem;
int iterations;
arma::Mat<unsigned char> dst;
void init()
{
src=arma::Mat<unsigned char>(n,m);
dst=arma::Mat<unsigned char>(n,m);
}
void erodeHelper(const arma::Mat<unsigned char>& src, arma::Mat<unsigned char>& dst, int erosion_elem, int iterations)
{
    int erosion_type = 0;
    if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }
    int erosion_size = erosion_elem;
    cv::Mat element = cv::getStructuringElement( erosion_type,
                            cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                            cv::Point( -1, -1 ) );
    cv::Mat srcCV;
    cv::Mat dstCV;
    srcCV = to_cvmat<unsigned char>(src);
    cv::erode( srcCV, dstCV, element, cv::Point(-1,-1), iterations );
    dst = to_arma<unsigned char>(dstCV);
}
void execute()
{
erodeHelper(src, dst, erosion_elem, iterations);
}

};
#endif
