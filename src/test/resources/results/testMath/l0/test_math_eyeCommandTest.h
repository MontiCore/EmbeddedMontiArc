/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_EYECOMMANDTEST
#define TEST_MATH_EYECOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_eyeCommandTest{
public:
void init()
{
}
void execute()
{
Matrix a = (Helper::getMatrixFromOctaveListFirstResult(Feye(Helper::convertToOctaveValueList(2, 2),1)));
}

};
#endif
