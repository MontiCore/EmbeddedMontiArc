/* (c) https://github.com/MontiCore/monticore */
#ifndef PAPER_MATHUNIT6_MATRIXMODIFIER_1_
#define PAPER_MATHUNIT6_MATRIXMODIFIER_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include <thread>
class paper_mathUnit6_matrixModifier_1_{
public:
Matrix mat1;
Matrix mat2;
Matrix mat3;
Matrix mat4;
Matrix mat5;
Matrix matOut;
void init()
{
mat1=Matrix(1000,2);
mat2=Matrix(2,1000);
mat3=Matrix(1000,2);
mat4=Matrix(2,10000);
mat5=Matrix(10000,10000);
matOut=Matrix(1000,10000);
}
void execute()
{
Matrix h1 = mat1*mat2;
Matrix h2 = mat3*mat4;
Matrix h3 = h1*h2;
matOut = h3*mat5;
}

};
#endif
