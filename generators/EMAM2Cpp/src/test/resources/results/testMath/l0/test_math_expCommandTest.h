/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_EXPCOMMANDTEST
#define TEST_MATH_EXPCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_expCommandTest{
public:
void init()
{
}
void execute()
{
double a = (Helper::getDoubleFromOctaveListFirstResult(Fexp(Helper::convertToOctaveValueList(5),1)));
}

};
#endif
