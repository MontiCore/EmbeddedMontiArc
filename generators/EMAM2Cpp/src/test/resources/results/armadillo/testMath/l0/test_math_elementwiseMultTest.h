/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ELEMENTWISEMULTTEST
#define TEST_MATH_ELEMENTWISEMULTTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_elementwiseMultTest{
public:
mat CONSTANTCONSTANTVECTOR0;
mat CONSTANTCONSTANTVECTOR1;
void init()
{
CONSTANTCONSTANTVECTOR0 = mat(2,2);
CONSTANTCONSTANTVECTOR0(0,0) = 1;
CONSTANTCONSTANTVECTOR0(0,1) = 1;
CONSTANTCONSTANTVECTOR0(1,0) = 1;
CONSTANTCONSTANTVECTOR0(1,1) = 1;
CONSTANTCONSTANTVECTOR1 = mat(2,2);
CONSTANTCONSTANTVECTOR1(0,0) = 1;
CONSTANTCONSTANTVECTOR1(0,1) = 2;
CONSTANTCONSTANTVECTOR1(1,0) = 3;
CONSTANTCONSTANTVECTOR1(1,1) = 4;
}
void execute()
{
mat A = CONSTANTCONSTANTVECTOR0;
mat B = CONSTANTCONSTANTVECTOR1;
mat C = A % B;
}

};
#endif
