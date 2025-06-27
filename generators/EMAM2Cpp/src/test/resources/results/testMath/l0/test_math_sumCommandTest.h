/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_SUMCOMMANDTEST
#define TEST_MATH_SUMCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_sumCommandTest{
public:
double in1;
double out1;
RowVector CONSTANTCONSTANTVECTOR0;
void init()
{
CONSTANTCONSTANTVECTOR0 = RowVector(3);
CONSTANTCONSTANTVECTOR0(0,0) = 1;
CONSTANTCONSTANTVECTOR0(0,1) = 2;
CONSTANTCONSTANTVECTOR0(0,2) = 3;
}
void execute()
{
RowVector mat = CONSTANTCONSTANTVECTOR0;
double a = (Helper::getDoubleFromOctaveListFirstResult(Fsum(Helper::convertToOctaveValueList(mat),1)));
out1 = a+in1;
}

};
#endif
