/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ATANHCOMMANDTEST
#define TEST_MATH_ATANHCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_atanhCommandTest{
public:
void init()
{
}
void execute()
{
double a = (atanh(0.5));
}

};
#endif
