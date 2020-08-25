/* (c) https://github.com/MontiCore/monticore */
#ifndef OPTIMIZER_COMPLEXVECTORMULTIPLICATION1
#define OPTIMIZER_COMPLEXVECTORMULTIPLICATION1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_complexVectorMultiplication1{
public:
Matrix mat1;
Matrix mat2;
Matrix mat3;
Matrix matOut;
void init()
{
mat1=Matrix(400,200);
mat2=Matrix(200,100);
mat3=Matrix(100,10);
matOut=Matrix(10,200);
}
void execute()
{
ColumnVector a = mat1.column(1-1);
RowVector b = mat2.row(1-1);
ColumnVector c = mat3.column(1-1);
matOut = a*(b*c);
}

};
#endif
