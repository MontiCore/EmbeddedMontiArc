/* (c) https://github.com/MontiCore/monticore */
#ifndef OPTIMIZER_COMPLEXVECTORMULTIPLICATION2
#define OPTIMIZER_COMPLEXVECTORMULTIPLICATION2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_complexVectorMultiplication2{
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
RowVector a = mat1.row(1-1);
ColumnVector b = mat2.column(1-1);
RowVector c = mat3.row(1-1);
matOut = a*b*c;
}

};
#endif
