/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_FLOATDIVISIONTEST
#define TEST_MATH_FLOATDIVISIONTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_floatDivisionTest{
public:
double out1;
mat out2;
mat CONSTANTCONSTANTVECTOR0;
void init()
{
out2=mat(2,2);
CONSTANTCONSTANTVECTOR0 = mat(2,2);
CONSTANTCONSTANTVECTOR0(0,0) = 1.0/4.0;
CONSTANTCONSTANTVECTOR0(0,1) = 1.0/4.0;
CONSTANTCONSTANTVECTOR0(1,0) = 1.0/4.0;
CONSTANTCONSTANTVECTOR0(1,1) = 1.0/4.0;
}
void execute()
{
out1 = 1.0/2.0;
mat A = CONSTANTCONSTANTVECTOR0;
out2 = A;
}

};
#endif
