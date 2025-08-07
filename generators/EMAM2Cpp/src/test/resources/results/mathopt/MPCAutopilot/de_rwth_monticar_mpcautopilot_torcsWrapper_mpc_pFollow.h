#ifndef DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER_MPC_PFOLLOW
#define DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER_MPC_PFOLLOW
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CallIpopt3114.h"
using namespace arma;
class de_rwth_monticar_mpcautopilot_torcsWrapper_mpc_pFollow{
public:
double angleOnTrackAxis;
double distanceFromTrackAxis;
colvec currentSpeed;
colvec wheelSpeeds;
double steering;
double gasPedal;
double brakePedal;
void init()
{
currentSpeed=colvec(3);
wheelSpeeds=colvec(4);
}
void execute()
{
double dT = 0.1;
double currentHeading = 0.5;
double currentDistance = 1.5;
double currentVelocity = 100;
colvec mpc_steering;
colvec mpc_acceleration;
double error;
CallIpopt3114::solveOptimizationProblemIpOpt(&mpc_steering, &mpc_acceleration, &error, angleOnTrackAxis, distanceFromTrackAxis, currentSpeed, wheelSpeeds, steering, gasPedal, brakePedal, dT, currentHeading, currentDistance, currentVelocity);
}

};
#endif
