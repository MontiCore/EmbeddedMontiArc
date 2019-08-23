/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_LOG2COMMANDTEST
#define TEST_MATH_LOG2COMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_log2CommandTest{
public:
void init()
{
}
void execute()
{
double a = (Helper::getDoubleFromOctaveListFirstResult(Flog2(Helper::convertToOctaveValueList(15),1)));
}

};
#endif
