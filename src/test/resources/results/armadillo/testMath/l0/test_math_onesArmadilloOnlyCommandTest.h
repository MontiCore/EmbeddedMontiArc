/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ONESARMADILLOONLYCOMMANDTEST
#define TEST_MATH_ONESARMADILLOONLYCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_onesArmadilloOnlyCommandTest{
public:
void init()
{
}
void execute()
{
colvec a = (ones<colvec>(3));
mat b = (ones<mat>(3, 3));
cube c = (ones<cube>(3, 3, 3));
}

};
#endif
