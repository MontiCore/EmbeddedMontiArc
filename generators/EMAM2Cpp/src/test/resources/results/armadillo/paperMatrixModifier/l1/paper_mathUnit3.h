#ifndef PAPER_MATHUNIT3
#define PAPER_MATHUNIT3
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "paper_mathUnit3_matrixModifier_1_.h"
using namespace arma;
class paper_mathUnit3{
public:
mat mat1[3];
mat mat2[3];
mat mat3[3];
mat mat4[3];
mat mat5[3];
mat matOut1[3];
paper_mathUnit3_matrixModifier_1_ matrixModifier[3];
void init()
{
mat1[0]=mat(1000,200);
mat1[1]=mat(1000,200);
mat1[2]=mat(1000,200);
mat2[0]=mat(1000,200);
mat2[1]=mat(1000,200);
mat2[2]=mat(1000,200);
mat3[0]=mat(200,10);
mat3[1]=mat(200,10);
mat3[2]=mat(200,10);
mat4[0]=mat(10,100);
mat4[1]=mat(10,100);
mat4[2]=mat(10,100);
mat5[0]=mat(100,50000);
mat5[1]=mat(100,50000);
mat5[2]=mat(100,50000);
matOut1[0]=mat(1000,100);
matOut1[1]=mat(1000,100);
matOut1[2]=mat(1000,100);
mat1[0]=mat(1000,200);
matrixModifier[0].mat1=mat(1000,2);
mat1[1]=mat(1000,200);
matrixModifier[1].mat1=mat(1000,2);
mat1[2]=mat(1000,200);
matrixModifier[2].mat1=mat(1000,2);
mat2[0]=mat(1000,200);
matrixModifier[0].mat2=mat(2,1000);
mat2[1]=mat(1000,200);
matrixModifier[1].mat2=mat(2,1000);
mat2[2]=mat(1000,200);
matrixModifier[2].mat2=mat(2,1000);
mat3[0]=mat(200,10);
matrixModifier[0].mat3=mat(1000,2);
mat3[1]=mat(200,10);
matrixModifier[1].mat3=mat(1000,2);
mat3[2]=mat(200,10);
matrixModifier[2].mat3=mat(1000,2);
mat4[0]=mat(10,100);
matrixModifier[0].mat4=mat(2,10000);
mat4[1]=mat(10,100);
matrixModifier[1].mat4=mat(2,10000);
mat4[2]=mat(10,100);
matrixModifier[2].mat4=mat(2,10000);
mat5[0]=mat(100,50000);
matrixModifier[0].mat5=mat(10000,10000);
mat5[1]=mat(100,50000);
matrixModifier[1].mat5=mat(10000,10000);
mat5[2]=mat(100,50000);
matrixModifier[2].mat5=mat(10000,10000);
matrixModifier[0].matOut=mat(1000,10000);
matOut1[0]=mat(1000,100);
matrixModifier[1].matOut=mat(1000,10000);
matOut1[1]=mat(1000,100);
matrixModifier[2].matOut=mat(1000,10000);
matOut1[2]=mat(1000,100);
matrixModifier[0].init();
matrixModifier[1].init();
matrixModifier[2].init();
}
void execute()
{
matrixModifier[0].mat1 = mat1[0];
matrixModifier[1].mat1 = mat1[1];
matrixModifier[2].mat1 = mat1[2];
matrixModifier[0].mat2 = mat2[0];
matrixModifier[1].mat2 = mat2[1];
matrixModifier[2].mat2 = mat2[2];
matrixModifier[0].mat3 = mat3[0];
matrixModifier[1].mat3 = mat3[1];
matrixModifier[2].mat3 = mat3[2];
matrixModifier[0].mat4 = mat4[0];
matrixModifier[1].mat4 = mat4[1];
matrixModifier[2].mat4 = mat4[2];
matrixModifier[0].mat5 = mat5[0];
matrixModifier[1].mat5 = mat5[1];
matrixModifier[2].mat5 = mat5[2];
matrixModifier[0].execute();
matOut1[0] = matrixModifier[0].matOut;
matrixModifier[1].execute();
matOut1[1] = matrixModifier[1].matOut;
matrixModifier[2].execute();
matOut1[2] = matrixModifier[2].matOut;
}

};
#endif
