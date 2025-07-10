/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_MATRIXCONSTANTVARIABLEMATHSECTIONTEST
#define TEST_MATH_MATRIXCONSTANTVARIABLEMATHSECTIONTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_matrixConstantVariableMathSectionTest{
public:
mat CONSTANTCONSTANTVECTOR0;
void init()
{
CONSTANTCONSTANTVECTOR0 = mat(2,2);
CONSTANTCONSTANTVECTOR0(0,0) = 1;
CONSTANTCONSTANTVECTOR0(0,1) = 0;
CONSTANTCONSTANTVECTOR0(1,0) = 0;
CONSTANTCONSTANTVECTOR0(1,1) = 1;
}
void execute()
{
mat a = CONSTANTCONSTANTVECTOR0;
}

};
#endif
