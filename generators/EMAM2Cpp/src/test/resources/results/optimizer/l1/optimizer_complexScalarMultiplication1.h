/* (c) https://github.com/MontiCore/monticore */
#ifndef OPTIMIZER_COMPLEXSCALARMULTIPLICATION1
#define OPTIMIZER_COMPLEXSCALARMULTIPLICATION1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_complexScalarMultiplication1{
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
Matrix a = mat1;
Matrix b = mat2;
matOut = a*(b*factor);
}

};
#endif
