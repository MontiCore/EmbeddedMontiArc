#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10
#define TESTING_SUBPACKAGE10_MYCOMPONENT10
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage10_myComponent10_c2cd.h"
#include "testing_subpackage10_myComponent10_behaviorGeneration.h"
#include "testing_subpackage10_myComponent10_motionPlanning.h"
class testing_subpackage10_myComponent10{
public:
double timeIncrement;
double currentVelocity;
double x;
double y;
double compass;
double currentEngine;
double currentSteering;
double currentBrakes;
double trajectory_length;
Matrix trajectory_x;
Matrix trajectory_y;
double engine;
double steering;
double brakes;
testing_subpackage10_myComponent10_c2cd c2cd;
testing_subpackage10_myComponent10_behaviorGeneration behaviorGeneration;
testing_subpackage10_myComponent10_motionPlanning motionPlanning;
void init()
{
trajectory_x=Matrix(1,100);
trajectory_y=Matrix(1,100);
c2cd.init();
behaviorGeneration.init();
motionPlanning.init();
}
void execute()
{
c2cd.compass = compass;
c2cd.execute();
behaviorGeneration.currentPositionX = x;
behaviorGeneration.currentPositionY = y;
behaviorGeneration.plannedTrajectoryLength = trajectory_length;
behaviorGeneration.plannedTrajectoryX = trajectory_x;
behaviorGeneration.plannedTrajectoryY = trajectory_y;
behaviorGeneration.currentDirectionX = c2cd.currentDirectionX;
behaviorGeneration.currentDirectionY = c2cd.currentDirectionY;
behaviorGeneration.execute();
motionPlanning.currentVelocity = currentVelocity;
motionPlanning.currentDirectionX = c2cd.currentDirectionX;
motionPlanning.currentDirectionY = c2cd.currentDirectionY;
motionPlanning.desiredDirectionX = behaviorGeneration.desiredDirectionX;
motionPlanning.desiredDirectionY = behaviorGeneration.desiredDirectionY;
motionPlanning.signedDistanceToTrajectory = behaviorGeneration.signedDistanceToTrajectory;
motionPlanning.desiredVelocity = behaviorGeneration.desiredVelocity;
motionPlanning.execute();
engine = motionPlanning.engine;
steering = motionPlanning.steering;
brakes = motionPlanning.brakes;
}

};
#endif
