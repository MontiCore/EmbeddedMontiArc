/* (c) https://github.com/MontiCore/monticore */
#ifndef OPTIMIZER_CHAINEDMULTIPLICATIONADDITION
#define OPTIMIZER_CHAINEDMULTIPLICATIONADDITION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_chainedMultiplicationAddition{
public:
Matrix mat1;
Matrix mat2;
Matrix mat3;
Matrix mat4;
Matrix matOut;
void init()
{
mat1=Matrix(10,200);
mat2=Matrix(10,10);
mat3=Matrix(200,10);
mat4=Matrix(200,10);
matOut=Matrix(10,200);
}
void execute()
{
matOut = mat3*mat1+mat4*mat1+mat3*mat2;
}

};
#endif
