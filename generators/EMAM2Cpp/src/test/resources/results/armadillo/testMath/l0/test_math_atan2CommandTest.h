/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ATAN2COMMANDTEST
#define TEST_MATH_ATAN2COMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_atan2CommandTest{
public:
void init()
{
}
void execute()
{
double a = (atan2(0.5));
}

};
#endif
