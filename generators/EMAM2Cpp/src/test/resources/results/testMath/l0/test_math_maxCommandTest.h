/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_MAXCOMMANDTEST
#define TEST_MATH_MAXCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_maxCommandTest{
public:
void init()
{
}
void execute()
{
double a = (Helper::getDoubleFromOctaveListFirstResult(Fmax(Helper::convertToOctaveValueList(15, 4),1)));
}

};
#endif
