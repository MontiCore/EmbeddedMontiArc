/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ABSCOMMANDTEST
#define TEST_MATH_ABSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_absCommandTest{
public:
void init()
{
}
void execute()
{
double a = (std::abs(-1));
}

};
#endif
