#ifndef OPTIMIZER_COMPLEXMATRIXRIGHTMULTIPLICATIONADDITION2
#define OPTIMIZER_COMPLEXMATRIXRIGHTMULTIPLICATIONADDITION2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_complexMatrixRightMultiplicationAddition2{
public:
Matrix mat1;
Matrix mat2;
Matrix mat4;
Matrix matOut;
void init()
{
mat1=Matrix(200,1000);
mat2=Matrix(200,200);
mat4=Matrix(200,1000);
matOut=Matrix(200,1000);
}
void execute()
{
Matrix a = mat1;
Matrix b = mat1;
Matrix c = a;
Matrix matd = b;
matOut = (mat2+mat4)*mat1;
}

};
#endif
