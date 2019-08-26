/* (c) https://github.com/MontiCore/monticore */
#ifndef OPTIMIZER_SIMPLEMATRIXACCESSLEFTMULTIPLICATIONADDITION
#define OPTIMIZER_SIMPLEMATRIXACCESSLEFTMULTIPLICATIONADDITION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_simpleMatrixAccessLeftMultiplicationAddition{
public:
Matrix mat1;
Matrix mat2;
Matrix mat4;
double valOut;
void init()
{
mat1=Matrix(1000,200);
mat2=Matrix(1000,200);
mat4=Matrix(10,100);
}
void execute()
{
matOut = mat1(2-1, 2-1)*(mat2(1-1, 1-1)+mat4(1-1, 1-1));
}

};
#endif
