/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_MAXCOMMANDTEST
#define TEST_MATH_MAXCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_maxCommandTest{
public:
void init()
{
}
void execute()
{
double a = (std::max(15, 4));
}

};
#endif
