/* (c) https://github.com/MontiCore/monticore */
#ifndef OPTIMIZER_SIMPLESCALARMULTIPLICATION1
#define OPTIMIZER_SIMPLESCALARMULTIPLICATION1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_simpleScalarMultiplication1{
public:
Matrix mat1;
Matrix mat2;
double factor;
Matrix matOut;
void init()
{
mat1=Matrix(400,200);
mat2=Matrix(200,100);
matOut=Matrix(10,200);
}
void execute()
{
matOut = mat1*(mat2*factor);
}

};
#endif
