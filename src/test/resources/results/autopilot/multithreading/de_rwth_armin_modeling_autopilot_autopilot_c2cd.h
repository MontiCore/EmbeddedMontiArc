#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_C2CD
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_C2CD
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_c2cd{
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
