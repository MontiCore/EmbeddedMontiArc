/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ZEROSCOMMANDTEST
#define TEST_MATH_ZEROSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_zerosCommandTest{
public:
void init()
{
}
void execute()
{
mat a = (zeros<mat>(3, 3));
}

};
#endif
