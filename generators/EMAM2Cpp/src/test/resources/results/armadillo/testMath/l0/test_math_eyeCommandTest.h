/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_EYECOMMANDTEST
#define TEST_MATH_EYECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_eyeCommandTest{
public:
void init()
{
}
void execute()
{
mat a = (eye(2, 2));
}

};
#endif
