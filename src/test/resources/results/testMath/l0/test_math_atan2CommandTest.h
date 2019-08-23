/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ATAN2COMMANDTEST
#define TEST_MATH_ATAN2COMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_atan2CommandTest{
public:
void init()
{
}
void execute()
{
double a = (Helper::getDoubleFromOctaveListFirstResult(Fatan2(Helper::convertToOctaveValueList(0.5),1)));
}

};
#endif
