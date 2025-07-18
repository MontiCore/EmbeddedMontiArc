#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_ENGINEANDBRAKES_PID
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_ENGINEANDBRAKES_PID
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_engineAndBrakes_pid{
public:
double paramP;
double paramI;
double paramD;
double paramDecayCoefficient;
double error;
double control;
bool isPrevErrorSpecified;
double prevError;
double acc;
void init()
{
isPrevErrorSpecified=0;
prevError=0;
acc=0;
}
void execute()
{
acc = paramDecayCoefficient*acc+error;
double drv = 0;
if(isPrevErrorSpecified){
drv = error-prevError;
}
control = paramP*error+paramI*acc+paramD*drv;
prevError = error;
isPrevErrorSpecified = 1;
}

};
#endif
