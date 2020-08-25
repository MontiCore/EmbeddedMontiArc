/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ONESCOMMANDTEST
#define TEST_MATH_ONESCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_onesCommandTest{
public:
void init()
{
}
void execute()
{
mat a = (ones<mat>(3, 3));
}

};
#endif
