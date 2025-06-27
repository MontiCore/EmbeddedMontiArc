/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_SUMEXTENDEDCOMMANDTEST
#define TEST_MATH_SUMEXTENDEDCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_sumExtendedCommandTest{
public:
colvec CONSTANTCONSTANTVECTOR0;
void init()
{
CONSTANTCONSTANTVECTOR0 = colvec(3);
CONSTANTCONSTANTVECTOR0(0,0) = 1;
CONSTANTCONSTANTVECTOR0(1,0) = 2;
CONSTANTCONSTANTVECTOR0(2,0) = 3;
}
double calcSum1(colvec A)
{
  double res = 0; 
  for (int i = 1; i <= 3; i++)
    res += A(i-1);
  return res;
}
double calcSum2(colvec A, double x, int j)
{
  double res = 0; 
  for (int i = 1; i <= 3; i++)
    res += A(i-1)*A(j-1);
  return res;
}
double calcSum3(colvec A, double x)
{
  double res = 0; 
  for (int j = 1; j <= 2; j++)
    res += calcSum2(A, x, j);
  return res;
}
double calcSum4(colvec A, double x, double y, int j, int k)
{
  double res = 0; 
  for (int i = 1; i <= 3; i++)
    res += A(i-1)*A(j-1)*A(k-1);
  return res;
}
double calcSum5(colvec A, double x, double y, int k)
{
  double res = 0; 
  for (int j = 1; j <= 2; j++)
    res += calcSum4(A, x, y, j, k);
  return res;
}
double calcSum6(colvec A, double x, double y)
{
  double res = 0; 
  for (int k = 1; k <= 1; k++)
    res += calcSum5(A, x, y, k);
  return res;
}
void execute()
{
colvec A = CONSTANTCONSTANTVECTOR0;
double x = calcSum1(A);
double y = calcSum3(A, x);
double z = calcSum6(A, x, y);
}

};
#endif
