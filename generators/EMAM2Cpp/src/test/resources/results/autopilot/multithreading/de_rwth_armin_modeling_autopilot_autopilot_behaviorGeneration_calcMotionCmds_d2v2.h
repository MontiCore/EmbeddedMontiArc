#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_BEHAVIORGENERATION_CALCMOTIONCMDS_D2V2
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_BEHAVIORGENERATION_CALCMOTIONCMDS_D2V2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration_calcMotionCmds_d2v2{
public:
double distance;
double minVelocity;
double maxVelocity;
double velocity;
double D1;
double V1;
double D2;
double V2;
double COEF_K;
void init()
{
D1=1;
V1=1;
D2=15;
V2=7;
COEF_K=(Helper::getDoubleFromOctaveListFirstResult(Flog(Helper::convertToOctaveValueList((V1/V2)),1)))/(D2-D1);
}
void execute()
{
double v = 0;
if(((distance < D1))){
v = 0;
}
else if(((distance >= D2))){
v = V2;
}
else {
v = V1*(Helper::getDoubleFromOctaveListFirstResult(Fexp(Helper::convertToOctaveValueList(-COEF_K*(distance-D1)),1)));
}
if(((v < minVelocity))){
v = minVelocity;
}
else if(((v > maxVelocity))){
v = maxVelocity;
}
velocity = v;
}

};
#endif
