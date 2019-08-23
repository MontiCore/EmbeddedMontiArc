/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_LOGCOMMANDTEST
#define TEST_MATH_LOGCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_logCommandTest{
public:
void init()
{
}
void execute()
{
double a = (Helper::getDoubleFromOctaveListFirstResult(Flog(Helper::convertToOctaveValueList(15),1)));
}

};
#endif
