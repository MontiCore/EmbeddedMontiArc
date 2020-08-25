/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_MATRIXASSIGNMENTTEST
#define TEST_MATH_MATRIXASSIGNMENTTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_matrixAssignmentTest{
public:
void init()
{
}
void execute()
{
mat matA = (ones<mat>(2, 2));
mat matB = (ones<mat>(2, 2));
matA(1-1, 1-1) = matB(1-1, 1-1);
}

};
#endif
