#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_ENGINEANDBRAKES_ABS1
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_ENGINEANDBRAKES_ABS1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_engineAndBrakes_abs1{
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
