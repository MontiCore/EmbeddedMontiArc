/* (c) https://github.com/MontiCore/monticore */
#ifndef OPTIMIZER_COMPLEXMATRIXLEFTMULTIPLICATIONADDITION
#define OPTIMIZER_COMPLEXMATRIXLEFTMULTIPLICATIONADDITION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_complexMatrixLeftMultiplicationAddition{
public:
Matrix mat1;
Matrix mat2;
Matrix mat4;
double factor;
Matrix matOut;
void init()
{
mat1=Matrix(1000,200);
mat2=Matrix(200,200);
mat4=Matrix(200,200);
matOut=Matrix(1000,200);
}
void execute()
{
matOut = mat1*(mat2+mat4);
}

};
#endif
