#ifndef TEST_MATH_DILATECOMMANDTEST
#define TEST_MATH_DILATECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace cv;
class test_math_dilateCommandTest{
public:
mat src;
int dilation_elem;
int iterations;
mat dst;
void init()
{
src=mat(n,m);
dst=mat(n,m);
}
void dilateHelper(mat src, mat dst, int dilation_elem, int iterations)
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
dilateHelper(src, dst, dilation_elem, iterations);
}

};
#endif