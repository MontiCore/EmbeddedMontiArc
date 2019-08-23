/* (c) https://github.com/MontiCore/monticore */
#ifndef PAPER_MATHUNIT_MATRIXMODIFIER_1_
#define PAPER_MATHUNIT_MATRIXMODIFIER_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class paper_mathUnit_matrixModifier_1_{
public:
Matrix mat1;
Matrix mat2;
Matrix mat3;
Matrix mat4;
Matrix mat5;
Matrix matOut;
void init()
{
mat1=Matrix(1000,200);
mat2=Matrix(1000,200);
mat3=Matrix(200,10);
mat4=Matrix(10,100);
mat5=Matrix(100,50000);
matOut=Matrix(1000,100);
}
void execute()
{
Matrix a = mat1+mat2;
Matrix b = mat3*mat4;
matOut = (a*b)*mat5;
}

};
#endif
