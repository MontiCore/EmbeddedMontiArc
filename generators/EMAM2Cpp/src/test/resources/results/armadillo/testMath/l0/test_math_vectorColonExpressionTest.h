/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_VECTORCOLONEXPRESSIONTEST
#define TEST_MATH_VECTORCOLONEXPRESSIONTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_vectorColonExpressionTest{
public:
void init()
{
}
void execute()
{
rowvec A = regspace<rowvec>(1, 1, 10);
rowvec B = regspace<rowvec>(1, 2, 10);
}

};
#endif
