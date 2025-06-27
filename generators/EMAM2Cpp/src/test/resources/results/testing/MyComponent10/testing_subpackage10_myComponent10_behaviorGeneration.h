/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_BEHAVIORGENERATION
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_BEHAVIORGENERATION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage10_myComponent10_behaviorGeneration_trimPath.h"
#include "testing_subpackage10_myComponent10_behaviorGeneration_calcMotionCmds.h"
class testing_subpackage10_myComponent10_behaviorGeneration{
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
testing_subpackage10_myComponent10_behaviorGeneration_trimPath trimPath;
testing_subpackage10_myComponent10_behaviorGeneration_calcMotionCmds calcMotionCmds;
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
