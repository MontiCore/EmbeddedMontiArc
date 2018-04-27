#ifndef PAPER_MATHUNIT1
#define PAPER_MATHUNIT1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "paper_mathUnit1_matrixModifier.h"
class paper_mathUnit1{
public:
Matrix mat1;
Matrix mat2;
Matrix mat3;
Matrix mat4;
double factor;
Matrix matOut1;
paper_mathUnit1_matrixModifier matrixModifier;
void init()
{
mat1=Matrix(1000,200);
mat2=Matrix(1000,200);
mat3=Matrix(200,10);
mat4=Matrix(10,100);
matOut1=Matrix(1000,100);
matrixModifier.init();
}
void execute()
{
matrixModifier.mat1 = mat1;
matrixModifier.mat2 = mat2;
matrixModifier.mat3 = mat3;
matrixModifier.mat4 = mat4;
matrixModifier.factor = factor;
matrixModifier.execute();
matOut1 = matrixModifier.matOut;
}

};
#endif
