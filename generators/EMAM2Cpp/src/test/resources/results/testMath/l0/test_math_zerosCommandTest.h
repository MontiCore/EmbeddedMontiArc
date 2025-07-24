/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_MATH_ZEROSCOMMANDTEST
#define TEST_MATH_ZEROSCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class test_math_zerosCommandTest{
public:
void init()
{
}
void execute()
{
Matrix a = (Helper::getMatrixFromOctaveListFirstResult(Fzeros(Helper::convertToOctaveValueList(3, 3),1)));
}

};
#endif
