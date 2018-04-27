#ifndef PAPER_MATHUNIT5_MATRIXMODIFIER_1_
#define PAPER_MATHUNIT5_MATRIXMODIFIER_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include <thread>
using namespace arma;
class paper_mathUnit5_matrixModifier_1_{
public:
mat mat1;
mat mat2;
mat mat3;
mat mat4;
double factor;
mat matOut;
void init()
{
mat1=mat(1000,200);
mat2=mat(1000,200);
mat3=mat(200,10);
mat4=mat(10,100);
matOut=mat(1000,100);
}
void execute()
{
mat a = mat1+mat2;
mat b = mat3*mat4;
matOut = (a*b)*factor;
}

};
#endif
