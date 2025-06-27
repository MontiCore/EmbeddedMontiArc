#ifndef PAPER_MATHUNIT
#define PAPER_MATHUNIT
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "paper_mathUnit_matrixModifier_1_.h"
using namespace arma;
class paper_mathUnit{
public:
mat mat1[2];
mat mat2[2];
mat mat3[2];
mat mat4[2];
mat mat5[2];
mat matOut1[2];
paper_mathUnit_matrixModifier_1_ matrixModifier[2];
void init()
{
mat1[0]=mat(1000,200);
mat1[1]=mat(1000,200);
mat2[0]=mat(1000,200);
mat2[1]=mat(1000,200);
mat3[0]=mat(200,10);
mat3[1]=mat(200,10);
mat4[0]=mat(10,100);
mat4[1]=mat(10,100);
mat5[0]=mat(100,50000);
mat5[1]=mat(100,50000);
matOut1[0]=mat(1000,100);
matOut1[1]=mat(1000,100);
matrixModifier[0].init();
matrixModifier[1].init();
}
void execute()
{
matrixModifier[0].mat1 = mat1[0];
matrixModifier[1].mat1 = mat1[1];
matrixModifier[0].mat2 = mat2[0];
matrixModifier[1].mat2 = mat2[1];
matrixModifier[0].mat3 = mat3[0];
matrixModifier[1].mat3 = mat3[1];
matrixModifier[0].mat4 = mat4[0];
matrixModifier[1].mat4 = mat4[1];
matrixModifier[0].mat5 = mat5[0];
matrixModifier[1].mat5 = mat5[1];
matrixModifier[0].execute();
matOut1[0] = matrixModifier[0].matOut;
matrixModifier[1].execute();
matOut1[1] = matrixModifier[1].matOut;
}

};
#endif
