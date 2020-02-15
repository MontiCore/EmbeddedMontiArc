#ifndef TEST_MATH_ERODEANDGAUSSIANBLURCOMMANDTEST
#define TEST_MATH_ERODEANDGAUSSIANBLURCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "ConvHelper.h"
#include "opencv2/imgproc.hpp"
#include <vector>
using namespace arma;
using namespace cv;
class test_math_erodeAndGaussianBlurCommandTest{
public:
cube src;
int erosion_elemIn;
colvec sizeY;
colvec two;
cube dst;
cube outMatrix;
mat out2Matrix;
void init()
{
src = cube(n, m, 3);
sizeY=colvec(3);
two=colvec(2);
dst = cube(n, m, 3);
outMatrix = cube(3, n, m);
out2Matrix=mat(2,m);
}
void erodeHelper(cube src, cube dst, int erosion_elem, int iterations)
{
    int erosion_type = 0;
    if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
    erosion_size = erosion_elem;
    mat element = getStructuringElement( erosion_type,
                            Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                            Point( -1, -1 ) );
    erode( src, dst, element, Point(-1,-1), iterations );
}
void dilateHelper(cube src, cube dst, int dilation_elem, int iterations)
{
    int dilation_type = 0;
    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
    dilation_size = dilation_elem;
    mat element = getStructuringElement( dilation_type,
                            Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                            Point( -1, -1 ) );
    dilate( src, dst, element, Point(-1,-1), iterations );
}
void execute()
{
erodeHelper(src, dst, erosion_elem, iterations);
cube dst2=cube(270,340,3);
erodeHelper(src, dst, erosion_elem, iterations);
erodeHelper(src, dst, erosion_elem, iterations);
dilateHelper(dst, dst3, dilation_elem, iterations);
cube src4 = (det(src));
erodeHelper(src4, dst, erosion_elem, iterations);
erodeHelper(src, dst, erosion_elem, iterations);
dilateHelper(dst, dst3, dilation_elem, iterations);
findContours(dst3, contours, method, mode);
}

};
#endif
