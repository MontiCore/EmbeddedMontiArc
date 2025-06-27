/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_MATRIXARRAYPORTACCESSTEST
#define TEST_MATH_MATRIXARRAYPORTACCESSTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_matrixArrayPortAccessTest{
public:
mat arrayIn[5];
void init()
{
arrayIn[0]=mat(3,3);
arrayIn[1]=mat(3,3);
arrayIn[2]=mat(3,3);
arrayIn[3]=mat(3,3);
arrayIn[4]=mat(3,3);
}
void execute()
{
int variableIII = 3;
mat tmpA = arrayIn[variableIII-1];
}

};
#endif
