/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_C2CD
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_C2CD
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class testing_subpackage10_myComponent10_c2cd{
public:
double compass;
double currentDirectionX;
double currentDirectionY;
void init()
{
}
void execute()
{
double angle = compass+0.5*M_PI;
currentDirectionX = (Helper::getDoubleFromOctaveListFirstResult(Fcos(Helper::convertToOctaveValueList(angle),1)));
currentDirectionY = (Helper::getDoubleFromOctaveListFirstResult(Fsin(Helper::convertToOctaveValueList(angle),1)));
}

};
#endif
