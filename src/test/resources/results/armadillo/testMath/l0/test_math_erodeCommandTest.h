#ifndef TEST_MATH_ERODECOMMANDTEST
#define TEST_MATH_ERODECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
#include "ConvHelper.h"
using namespace arma;
using namespace std;
class test_math_erodeCommandTest{
public:
arma::mat src;
int erosion_elem;
int iterations;
arma::mat dst;
void init()
{
src=mat(n,m);
dst=mat(n,m);
}
void erodeHelper(arma::mat src, arma::mat dst, int erosion_elem, int iterations)
{
    int erosion_type = 0;
    if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
    erosion_size = erosion_elem;
    cv::Mat element = cv::getStructuringElement( erosion_type,
                            Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                            Point( -1, -1 ) );
    cv::Mat srcCV;
    cv::Mat dstCV;
    srcCV = ConvHelper::to_cvmat(src);
    cv::erode( srcCV, dstCV, element, Point(-1,-1), iterations );
    dst = ConvHelper::to_arma(dstCV);
}
void execute()
{
erodeHelper(src, dst, erosion_elem, iterations);
}

};
#endif
