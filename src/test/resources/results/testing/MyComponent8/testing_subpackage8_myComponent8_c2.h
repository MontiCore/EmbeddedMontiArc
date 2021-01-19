/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE8_MYCOMPONENT8_C2
#define TESTING_SUBPACKAGE8_MYCOMPONENT8_C2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class testing_subpackage8_myComponent8_c2{
public:
double in1;
double in2;
double out1;
void init()
{
}
void execute()
{
out1 = (Helper::getDoubleFromOctaveListFirstResult(Fmin(Helper::convertToOctaveValueList(in1*in1+in2*in2, 100),1)));
}

};
#endif
