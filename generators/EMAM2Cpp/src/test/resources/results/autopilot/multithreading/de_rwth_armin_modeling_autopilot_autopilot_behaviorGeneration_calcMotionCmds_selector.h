#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_BEHAVIORGENERATION_CALCMOTIONCMDS_SELECTOR
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_BEHAVIORGENERATION_CALCMOTIONCMDS_SELECTOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration_calcMotionCmds_selector{
public:
bool isDriveToFirstPosition;
double desiredDirectionX1;
double desiredDirectionY1;
double signedDistanceToTrajectory1;
double desiredVelocity1;
double desiredDirectionX2;
double desiredDirectionY2;
double signedDistanceToTrajectory2;
double desiredVelocity2;
double desiredDirectionX;
double desiredDirectionY;
double signedDistanceToTrajectory;
double desiredVelocity;
void init()
{
}
void execute()
{
if((isDriveToFirstPosition)){
desiredDirectionX = desiredDirectionX1;
desiredDirectionY = desiredDirectionY1;
signedDistanceToTrajectory = signedDistanceToTrajectory1;
desiredVelocity = desiredVelocity1;
}
else {
desiredDirectionX = desiredDirectionX2;
desiredDirectionY = desiredDirectionY2;
signedDistanceToTrajectory = signedDistanceToTrajectory2;
desiredVelocity = desiredVelocity2;
}
}

};
#endif
