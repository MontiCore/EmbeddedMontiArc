#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_ENGINEANDBRAKES_PIDERROR
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_ENGINEANDBRAKES_PIDERROR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_engineAndBrakes_pidError{
public:
double currentVelocity;
double desiredVelocity;
double error;
double V_THRESHOLD_FOR_ERROR_LEAP;
void init()
{
V_THRESHOLD_FOR_ERROR_LEAP=1.5;
}
void execute()
{
if(((desiredVelocity <= 0.01))){
error = -100;
}
else if((((currentVelocity-desiredVelocity) > V_THRESHOLD_FOR_ERROR_LEAP))){
error = -currentVelocity;
}
else if((((desiredVelocity-currentVelocity) > V_THRESHOLD_FOR_ERROR_LEAP))){
error = desiredVelocity;
}
else {
error = desiredVelocity-currentVelocity;
}
}

};
#endif
