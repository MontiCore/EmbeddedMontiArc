#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_ENGINEANDBRAKES_PIDPARAMS
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_ENGINEANDBRAKES_PIDPARAMS
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_engineAndBrakes_pidParams{
public:
double currentVelocity;
double desiredVelocity;
double paramP;
double paramI;
double paramD;
double paramDecayCoefficient;
double MIN_VELOCITY;
double MAX_VELOCITY;
double P_FOR_MIN_VELOCITY;
double P_FOR_MAX_VELOCITY;
void init()
{
MIN_VELOCITY=0.5;
MAX_VELOCITY=13-MIN_VELOCITY/10;
P_FOR_MIN_VELOCITY=1.76703;
P_FOR_MAX_VELOCITY=3.29578;
}
void execute()
{
paramP = 0;
paramI = 0;
paramD = 0;
paramDecayCoefficient = 0;
double v = currentVelocity;
if(((desiredVelocity < currentVelocity))){
v = desiredVelocity;
}
if(((v < MIN_VELOCITY))){
paramP = P_FOR_MIN_VELOCITY;
}
else if(((v > MAX_VELOCITY))){
paramP = P_FOR_MAX_VELOCITY;
}
else {
double diff = v-MIN_VELOCITY;
paramP = P_FOR_MIN_VELOCITY+(P_FOR_MAX_VELOCITY-P_FOR_MIN_VELOCITY)*diff/(MAX_VELOCITY-MIN_VELOCITY);
}
}

};
#endif
