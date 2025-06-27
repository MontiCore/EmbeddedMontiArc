/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ARMADILLOINDEXTEST
#define TEST_MATH_ARMADILLOINDEXTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_math_armadilloIndexTest{
public:
mat in1;
int inRow;
int inCol;
mat out1;
mat CONSTANTCONSTANTVECTOR0;
rowvec CONSTANTCONSTANTVECTOR1;
colvec CONSTANTCONSTANTVECTOR2;
void init()
{
in1=mat(2,2);
out1=mat(4,4);
CONSTANTCONSTANTVECTOR0 = mat(2,2);
CONSTANTCONSTANTVECTOR0(0,0) = 1;
CONSTANTCONSTANTVECTOR0(0,1) = 2;
CONSTANTCONSTANTVECTOR0(1,0) = 3;
CONSTANTCONSTANTVECTOR0(1,1) = 4;
CONSTANTCONSTANTVECTOR1 = rowvec(2);
CONSTANTCONSTANTVECTOR1(0,0) = 11;
CONSTANTCONSTANTVECTOR1(0,1) = 12;
CONSTANTCONSTANTVECTOR2 = colvec(4);
CONSTANTCONSTANTVECTOR2(0,0) = 1;
CONSTANTCONSTANTVECTOR2(1,0) = 1;
CONSTANTCONSTANTVECTOR2(2,0) = 1;
CONSTANTCONSTANTVECTOR2(3,0) = 1;
}
void execute()
{
mat A=mat(2,2);
A = (ones<mat>(2, 2));
A = CONSTANTCONSTANTVECTOR0;
colvec b=colvec(2);
b = (zeros<colvec>(2));
b = CONSTANTCONSTANTVECTOR1;
out1 = (zeros<mat>(4, 4));
A(2-1, 2-1) = 5;
A(2-1) = b(2-1);
A(1-1, 1-1) = b(2-1);
A(2-1, 1-1) = out1(1-1, 1-1);
b(2-1) = 13;
b(1-1) = A(1-1);
b(1-1) = A(2-1, 2-1);
b(2-1) = out1(4-1, 4-1);
double x = 0;
x = A(1-1, 2-1);
x = b(2-1);
x = in1(1-1, 1-1);
out1(1-1, 1-1) = in1(1-1, 1-1);
mat test = (zeros<mat>(4, 4));
out1 = test;
out1(1-1) = test(1-1);
out1(2-1, 2-1) = A(2-1, 2-1);
double y = A(inRow-1, inCol-1);
colvec C = CONSTANTCONSTANTVECTOR2;
y = C(1-1)*C(4-1)*(C(1-1)+C(2-1)+C(3-1))+C(3-1);
}

};
#endif
