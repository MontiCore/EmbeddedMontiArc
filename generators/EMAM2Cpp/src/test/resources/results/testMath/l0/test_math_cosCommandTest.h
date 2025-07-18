/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_COSCOMMANDTEST
#define TEST_MATH_COSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_cosCommandTest{
public:
void init()
{
}
void execute()
{
double a = (Helper::getDoubleFromOctaveListFirstResult(Fcos(Helper::convertToOctaveValueList(0.5),1)));
}

};
#endif
