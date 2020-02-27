#ifndef TEST_MATH_ERODEANDGAUSSIANBLURCOMMANDTEST
#define TEST_MATH_ERODEANDGAUSSIANBLURCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
#include "ConvHelper.h"
#include <vector>
using namespace arma;
using namespace std;
class test_math_erodeAndGaussianBlurCommandTest{
public:
Cube<unsigned char> src;
int erosion_elemIn;
colvec sizeY;
colvec two;
Cube<unsigned char> dst;
Cube<unsigned char> outMatrix;
arma::Mat<unsigned char> out2Matrix;
void init()
{
src = cube(n, m, 3);
sizeY=colvec(3);
two=colvec(2);
dst = cube(n, m, 3);
outMatrix = cube(3, n, m);
out2Matrix=Mat<unsigned char>(2,m);
}
void erodeHelper(const Cube<unsigned char>& src, cv::Mat& dst, int erosion_elem, int iterations)
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
    srcCV = to_cvmat<unsigned char>(src);
    cv::erode( srcCV, dst, element, cv::Point(-1,-1), iterations );
}
void dilateHelper(const cv::Mat& src, cv::Mat& dst, int dilation_elem, int iterations)
{
    int dilation_type = 0;
    if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }
    int dilation_size = dilation_elem;
    cv::Mat element = cv::getStructuringElement( dilation_type,
                            cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                            cv::Point( -1, -1 ) );
    cv::dilate( src, dst, element, cv::Point(-1,-1), iterations );
}
void findContoursHelper(const cv::Mat& image, std::vector<std::vector<cv::Point>>& contours, int mode, int method)
{
    cv::findContours( image, contours, mode, method );
}
void execute()
{
erodeHelper(src, dst, erosion_elem, iterations);
cube dst2=cube(270,340,3);
erodeHelper(src, dst2, erosion_elem, iterations);
erodeHelper(src, dst, erosion_elem, iterations);
dilateHelper(dst, dst3, dilation_elem, iterations);
cube src4 = (det(src));
erodeHelper(src4, dst, erosion_elem, iterations);
erodeHelper(src, dst, erosion_elem, iterations);
dilateHelper(dst, dst3, dilation_elem, iterations);
findContoursHelper(dst3, contours, method, mode);
}

};
#endif
