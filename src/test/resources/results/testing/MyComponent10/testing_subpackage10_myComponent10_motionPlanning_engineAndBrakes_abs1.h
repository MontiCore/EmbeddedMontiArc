/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_ENGINEANDBRAKES_ABS1
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_ENGINEANDBRAKES_ABS1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_abs1{
public:
double input;
double output;
void init()
{
}
void execute()
{
output = (Helper::getDoubleFromOctaveListFirstResult(Fabs(Helper::convertToOctaveValueList(input),1)));
}

};
#endif
