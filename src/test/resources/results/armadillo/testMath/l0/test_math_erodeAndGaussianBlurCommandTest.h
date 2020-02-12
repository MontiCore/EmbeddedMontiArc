#ifndef TEST_MATH_ERODEANDGAUSSIANBLURCOMMANDTEST
#define TEST_MATH_ERODEANDGAUSSIANBLURCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
#include <vector>
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace cv;
class test_math_erodeAndGaussianBlurCommandTest{
public:
cube src;
int erosion_elemIn;
colvec sizeY;
colvec two;
cube outMatrix;
mat out2Matrix;
void init()
{
src = cube(3, n, m);
sizeY=colvec(3);
two=colvec(2);
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
void execute()
{
findContours(image, contours, mode, method);
erodeHelper(src, dst, erosion_elem, iterations);
}

};
#endif
