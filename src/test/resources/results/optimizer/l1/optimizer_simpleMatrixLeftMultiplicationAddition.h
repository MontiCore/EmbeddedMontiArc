/* (c) https://github.com/MontiCore/monticore */
#ifndef OPTIMIZER_SIMPLEMATRIXLEFTMULTIPLICATIONADDITION
#define OPTIMIZER_SIMPLEMATRIXLEFTMULTIPLICATIONADDITION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_simpleMatrixLeftMultiplicationAddition{
public:
Matrix mat1;
Matrix mat2;
Matrix mat4;
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
