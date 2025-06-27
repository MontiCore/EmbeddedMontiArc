#ifndef OPTIMIZER_SIMPLESCALARMULTIPLICATION2
#define OPTIMIZER_SIMPLESCALARMULTIPLICATION2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class optimizer_simpleScalarMultiplication2{
public:
Matrix mat1;
Matrix mat2;
double factor;
Matrix matOut;
void init()
{
mat1=Matrix(100,200);
mat2=Matrix(200,800);
matOut=Matrix(10,200);
}
void execute()
{
matOut = mat1*(mat2*factor);
}

};
#endif
