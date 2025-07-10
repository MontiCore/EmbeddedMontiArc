/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_MATSCALARADDTEST
#define TEST_MATH_MATSCALARADDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_matScalarAddTest{
public:
void init()
{
}
void execute()
{
mat A = (ones<mat>(2, 2));
double b = 1;
A = A+1;
}

};
#endif
