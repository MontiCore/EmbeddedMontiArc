/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_LOG10COMMANDTEST
#define TEST_MATH_LOG10COMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_log10CommandTest{
public:
void init()
{
}
void execute()
{
double a = (Helper::getDoubleFromOctaveListFirstResult(Flog10(Helper::convertToOctaveValueList(15),1)));
}

};
#endif
