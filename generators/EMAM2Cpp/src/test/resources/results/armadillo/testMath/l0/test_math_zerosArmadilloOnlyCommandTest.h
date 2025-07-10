/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ZEROSARMADILLOONLYCOMMANDTEST
#define TEST_MATH_ZEROSARMADILLOONLYCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_zerosArmadilloOnlyCommandTest{
public:
void init()
{
}
void execute()
{
colvec a = (zeros<colvec>(3));
mat b = (zeros<mat>(3, 3));
cube c = (zeros<cube>(3, 3, 3));
}

};
#endif
