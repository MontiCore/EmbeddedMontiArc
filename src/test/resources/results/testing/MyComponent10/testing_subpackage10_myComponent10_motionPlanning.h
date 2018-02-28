#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage10_myComponent10_motionPlanning_keepDirection.h"
#include "testing_subpackage10_myComponent10_motionPlanning_sac.h"
#include "testing_subpackage10_myComponent10_motionPlanning_sum1.h"
#include "testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes.h"
#include "testing_subpackage10_myComponent10_motionPlanning_boundsSteering.h"
#include "testing_subpackage10_myComponent10_motionPlanning_boundsBrakes.h"
#include "testing_subpackage10_myComponent10_motionPlanning_boundsEngine.h"
class testing_subpackage10_myComponent10_motionPlanning{
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
testing_subpackage10_myComponent10_motionPlanning_keepDirection keepDirection;
testing_subpackage10_myComponent10_motionPlanning_sac sac;
testing_subpackage10_myComponent10_motionPlanning_sum1 sum1;
testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes engineAndBrakes;
testing_subpackage10_myComponent10_motionPlanning_boundsSteering boundsSteering;
testing_subpackage10_myComponent10_motionPlanning_boundsBrakes boundsBrakes;
testing_subpackage10_myComponent10_motionPlanning_boundsEngine boundsEngine;
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
sac.signedDistanceToTrajectory = signedDistanceToTrajectory;
sac.execute();
sum1.t1 = keepDirection.steeringAngle;
sum1.t2 = sac.steeringAngleCorrection;
sum1.execute();
keepDirection.currentDirectionX = currentDirectionX;
keepDirection.currentDirectionY = currentDirectionY;
keepDirection.desiredDirectionX = desiredDirectionX;
keepDirection.desiredDirectionY = desiredDirectionY;
keepDirection.execute();
engineAndBrakes.currentVelocity = currentVelocity;
engineAndBrakes.desiredVelocity = desiredVelocity;
engineAndBrakes.execute();
boundsSteering.input = sum1.result;
boundsSteering.execute();
boundsBrakes.input = engineAndBrakes.brakes;
boundsBrakes.execute();
boundsEngine.input = engineAndBrakes.engine;
boundsEngine.execute();
engine = boundsEngine.output;
steering = boundsSteering.output;
brakes = boundsBrakes.output;
}

};
#endif
