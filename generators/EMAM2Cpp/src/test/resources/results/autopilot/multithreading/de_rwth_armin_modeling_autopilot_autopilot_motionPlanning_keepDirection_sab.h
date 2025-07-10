#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_KEEPDIRECTION_SAB
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_KEEPDIRECTION_SAB
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_keepDirection_sab{
public:
double v1x;
double v1y;
double v2x;
double v2y;
double angle;
double EPSILON;
void init()
{
EPSILON=0.001;
}
void execute()
{
angle = 0;
double norm1 = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(v1x*v1x+v1y*v1y),1)));
double norm2 = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(v2x*v2x+v2y*v2y),1)));
if((((norm1 > EPSILON))&&((norm2 > EPSILON)))){
double angle1 = -(Helper::getDoubleFromOctaveListFirstResult(Fatan2(Helper::convertToOctaveValueList(v1y, v1x),1)));
double c = (Helper::getDoubleFromOctaveListFirstResult(Fcos(Helper::convertToOctaveValueList(angle1),1)));
double s = (Helper::getDoubleFromOctaveListFirstResult(Fsin(Helper::convertToOctaveValueList(angle1),1)));
double v2xr = v2x*c-v2y*s;
double v2yr = v2x*s+v2y*c;
double angle2 = -(Helper::getDoubleFromOctaveListFirstResult(Fatan2(Helper::convertToOctaveValueList(v2yr, v2xr),1)));
double abs1 = (Helper::getDoubleFromOctaveListFirstResult(Fabs(Helper::convertToOctaveValueList(angle2),1)));
double abs2 = (Helper::getDoubleFromOctaveListFirstResult(Fabs(Helper::convertToOctaveValueList(abs1-M_PI),1)));
if(((abs2 <= EPSILON))){
angle = -M_PI;
}
else {
angle = angle2;
}
}
}

};
#endif
