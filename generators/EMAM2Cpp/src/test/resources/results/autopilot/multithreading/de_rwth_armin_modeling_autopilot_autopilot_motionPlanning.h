#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_keepDirection.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_sac.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_sum1.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_engineAndBrakes.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsSteering.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsBrakes.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsEngine.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_motionPlanning{
public:
double currentDirectionX;
double currentDirectionY;
double desiredDirectionX;
double desiredDirectionY;
double signedDistanceToTrajectory;
double currentVelocity;
double desiredVelocity;
double engine;
double steering;
double brakes;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_keepDirection keepDirection;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_sac sac;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_sum1 sum1;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_engineAndBrakes engineAndBrakes;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsSteering boundsSteering;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsBrakes boundsBrakes;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsEngine boundsEngine;
void init()
{
keepDirection.init();
sac.init();
sum1.init();
engineAndBrakes.init();
boundsSteering.init();
boundsBrakes.init();
boundsEngine.init();
}
void execute()
{
}

};
#endif