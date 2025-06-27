/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_STATICMATHSECTIONVARIABLETEST
#define TEST_MATH_STATICMATHSECTIONVARIABLETEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_staticMathSectionVariableTest{
public:
double counter;
void init()
{
counter=2;
}
void execute()
{
}

};
#endif
