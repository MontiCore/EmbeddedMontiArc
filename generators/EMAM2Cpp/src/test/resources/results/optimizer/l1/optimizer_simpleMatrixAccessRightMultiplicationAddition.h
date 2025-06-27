/* (c) https://github.com/MontiCore/monticore */
#ifndef OPTIMIZER_SIMPLEMATRIXACCESSRIGHTMULTIPLICATIONADDITION
#define OPTIMIZER_SIMPLEMATRIXACCESSRIGHTMULTIPLICATIONADDITION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_simpleMatrixAccessRightMultiplicationAddition{
public:
Matrix mat1;
Matrix mat2;
Matrix mat3;
double valOut;
void init()
{
mat1=Matrix(1000,200);
mat2=Matrix(1000,200);
mat3=Matrix(200,10);
}
void execute()
{
matOut = (mat2(5-1, 1-1)+mat3(8-1, 4-1))*mat1(1-1, 7-1);
}

};
#endif
