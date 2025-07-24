#ifndef PAPER_MATHUNIT_MATRIXMODIFIER_1_
#define PAPER_MATHUNIT_MATRIXMODIFIER_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class paper_mathUnit_matrixModifier_1_{
public:
mat mat1;
mat mat2;
mat mat3;
mat mat4;
mat mat5;
mat matOut;
void init()
{
mat1=mat(1000,2);
mat2=mat(2,1000);
mat3=mat(1000,2);
mat4=mat(2,10000);
mat5=mat(10000,10000);
matOut=mat(1000,10000);
}
void execute()
{
mat h1 = mat1*mat2;
mat h2 = mat3*mat4;
mat h3 = h1*h2;
matOut = h3*mat5;
}

};
#endif
