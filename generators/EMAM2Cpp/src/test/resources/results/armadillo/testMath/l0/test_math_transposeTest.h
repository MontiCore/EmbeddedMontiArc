/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_TRANSPOSETEST
#define TEST_MATH_TRANSPOSETEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_transposeTest{
public:
void init()
{
}
void execute()
{
mat a = (zeros<mat>(3, 2));
mat b = a.t();
}

};
#endif
