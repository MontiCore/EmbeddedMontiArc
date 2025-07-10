/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_GCDCOMMANDTEST
#define TEST_MATH_GCDCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_gcdCommandTest{
public:
void init()
{
}
void execute()
{
RowVector a = (Helper::getDoubleFromOctaveListFirstResult(Fgcd(Helper::convertToOctaveValueList(10, 8),1)));
}

};
#endif
