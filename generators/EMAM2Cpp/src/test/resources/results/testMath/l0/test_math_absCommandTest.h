/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ABSCOMMANDTEST
#define TEST_MATH_ABSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_absCommandTest{
public:
void init()
{
}
void execute()
{
double a = (Helper::getDoubleFromOctaveListFirstResult(Fabs(Helper::convertToOctaveValueList(-1),1)));
}

};
#endif
