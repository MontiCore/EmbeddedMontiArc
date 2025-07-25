#ifndef TEST_MATH_SCALECUBECOMMANDTEST
#define TEST_MATH_SCALECUBECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_scaleCubeCommandTest{
public:
cube img_in;
cube img_out;
void init()
{
img_in = cube(1, 28, 28);
img_out = cube(1, 64, 64);
}
cube scaleCube1(cube img, int depth_axis, int new_x, int new_y)
{
    if (depth_axis == 0) { 
        img = arma::reshape(img, img.n_cols, img.n_slices, img.n_rows);
    } else if(depth_axis == 1) {
        img = arma::reshape(img, img.n_rows, img.n_slices, img.n_cols);
    }
    
    arma::cube r_img = arma::cube(64,64, img.n_slices);
    for (int i = 0; i < img.n_slices; i++) 
    {
        arma::mat cur_slice = img.slice(i);
        arma::vec X = arma::regspace(0, cur_slice.n_cols-1);
        arma::vec Y = arma::regspace(0, cur_slice.n_rows-1);

        float scale_x = (cur_slice.n_cols-1)/float((new_x));
        float scale_y = (cur_slice.n_rows-1)/float((new_y));
        arma::vec XI = arma::regspace(0, new_x-1) * scale_x;
        arma::vec YI = arma::regspace(0, new_y-1) * scale_y;

        arma::mat mat_out;

        arma::interp2(X, Y, cur_slice, XI, YI, mat_out);
        r_img.slice(i) = mat_out;
    }

    if (depth_axis == 0) {
        r_img = arma::reshape(r_img, r_img.n_slices, r_img.n_rows, r_img.n_cols);
    } else if (depth_axis == 1) {
        r_img = arma::reshape(r_img, r_img.n_rows, r_img.n_slices, r_img.n_cols);
    }
    
    return r_img;
}
void execute()
{
img_out = scaleCube1(img_in, 0, 64, 64);
}

};
#endif
