#ifndef TEST_MATH_JOINCUBEDIMCOMMANDTEST
#define TEST_MATH_JOINCUBEDIMCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_joinCubeDimCommandTest{
public:
cube img_in;
cube img_out;
void init()
{
img_in = cube(1, 28, 28);
img_out = cube(2, 28, 28);
}
cube joinCubeDim1(cube c1, cube c2, int dim)
{
    if (dim == 0) {
        c1.insert_rows(c1.n_rows, c2);
        return c1;
    } else if(dim == 1) {
        c1.insert_cols(c1.n_cols, c2);
        return c1;
    }

    c1 = arma::join_slices(c1, c2);
    return c1;}
void execute()
{
img_out = joinCubeDim1(img_in, img_in, 0);
}

};
#endif
