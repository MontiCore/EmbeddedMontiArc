/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ACOSHCOMMANDTEST
#define TEST_MATH_ACOSHCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_acoshCommandTest{
public:
void init()
{
}
void execute()
{
double a = (acosh(0.5));
}

};
#endif
