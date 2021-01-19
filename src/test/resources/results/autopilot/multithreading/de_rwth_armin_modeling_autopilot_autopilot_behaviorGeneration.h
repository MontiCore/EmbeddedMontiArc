#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_BEHAVIORGENERATION
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_BEHAVIORGENERATION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration_trimPath.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration_calcMotionCmds.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration{
public:
double currentPositionX;
double currentPositionY;
double currentDirectionX;
double currentDirectionY;
int plannedTrajectoryLength;
Matrix plannedTrajectoryX;
Matrix plannedTrajectoryY;
double desiredDirectionX;
double desiredDirectionY;
double signedDistanceToTrajectory;
double desiredVelocity;
de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration_trimPath trimPath;
de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration_calcMotionCmds calcMotionCmds;
void init()
{
plannedTrajectoryX=Matrix(1,100);
plannedTrajectoryY=Matrix(1,100);
trimPath.init();
calcMotionCmds.init();
}
void execute()
{
}

};
#endif
