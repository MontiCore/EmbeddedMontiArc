/* (c) https://github.com/MontiCore/monticore */
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
int trajectory_length;
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
motionPlanning.engineAndBrakes.pidParams.currentVelocity = currentVelocity;
motionPlanning.engineAndBrakes.pidError.currentVelocity = currentVelocity;
behaviorGeneration.trimPath.currentPositionX = x;
behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.currentPositionX = x;
behaviorGeneration.calcMotionCmds.driveToFirstPosition.currentPositionX = x;
behaviorGeneration.calcMotionCmds.followTrajectory.currentPositionX = x;
behaviorGeneration.trimPath.currentPositionY = y;
behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.currentPositionY = y;
behaviorGeneration.calcMotionCmds.driveToFirstPosition.currentPositionY = y;
behaviorGeneration.calcMotionCmds.followTrajectory.currentPositionY = y;
c2cd.compass = compass;
behaviorGeneration.trimPath.plannedTrajectoryLength = trajectory_length;
behaviorGeneration.trimPath.plannedTrajectoryX = trajectory_x;
behaviorGeneration.trimPath.plannedTrajectoryY = trajectory_y;
motionPlanning.boundsSteering.eb.lowerBound = -0.785;
motionPlanning.boundsSteering.eb.upperBound = 0.785;
motionPlanning.boundsBrakes.eb.lowerBound = 0;
motionPlanning.boundsBrakes.eb.upperBound = 3;
motionPlanning.boundsEngine.eb.lowerBound = 0;
motionPlanning.boundsEngine.eb.upperBound = 2.5;
c2cd.execute();
motionPlanning.keepDirection.sab.v1x = c2cd.currentDirectionX;
motionPlanning.keepDirection.sab.v1y = c2cd.currentDirectionY;
behaviorGeneration.trimPath.execute();
behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.trimmedTrajectoryLength = behaviorGeneration.trimPath.trimmedTrajectoryLength;
behaviorGeneration.calcMotionCmds.driveToFirstPosition.trimmedTrajectoryLength = behaviorGeneration.trimPath.trimmedTrajectoryLength;
behaviorGeneration.calcMotionCmds.followTrajectory.trimmedTrajectoryLength = behaviorGeneration.trimPath.trimmedTrajectoryLength;
behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.trimmedTrajectoryX = behaviorGeneration.trimPath.trimmedTrajectoryX;
behaviorGeneration.calcMotionCmds.driveToFirstPosition.trimmedTrajectoryX = behaviorGeneration.trimPath.trimmedTrajectoryX;
behaviorGeneration.calcMotionCmds.followTrajectory.trimmedTrajectoryX = behaviorGeneration.trimPath.trimmedTrajectoryX;
behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.trimmedTrajectoryY = behaviorGeneration.trimPath.trimmedTrajectoryY;
behaviorGeneration.calcMotionCmds.driveToFirstPosition.trimmedTrajectoryY = behaviorGeneration.trimPath.trimmedTrajectoryY;
behaviorGeneration.calcMotionCmds.followTrajectory.trimmedTrajectoryY = behaviorGeneration.trimPath.trimmedTrajectoryY;
behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.execute();
behaviorGeneration.calcMotionCmds.driveToFirstPosition.isDriveToFirstPosition = behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.result;
behaviorGeneration.calcMotionCmds.followTrajectory.isDriveToFirstPosition = behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.result;
behaviorGeneration.calcMotionCmds.selector.isDriveToFirstPosition = behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.result;
behaviorGeneration.calcMotionCmds.driveToFirstPosition.execute();
behaviorGeneration.calcMotionCmds.selector.desiredDirectionX1 = behaviorGeneration.calcMotionCmds.driveToFirstPosition.desiredDirectionX;
behaviorGeneration.calcMotionCmds.selector.desiredDirectionY1 = behaviorGeneration.calcMotionCmds.driveToFirstPosition.desiredDirectionY;
behaviorGeneration.calcMotionCmds.selector.signedDistanceToTrajectory1 = behaviorGeneration.calcMotionCmds.driveToFirstPosition.signedDistanceToTrajectory;
behaviorGeneration.calcMotionCmds.d2v1.minVelocity = behaviorGeneration.calcMotionCmds.driveToFirstPosition.minVelocity;
behaviorGeneration.calcMotionCmds.d2v1.maxVelocity = behaviorGeneration.calcMotionCmds.driveToFirstPosition.maxVelocity;
behaviorGeneration.calcMotionCmds.d2v1.distance = behaviorGeneration.calcMotionCmds.driveToFirstPosition.distance;
behaviorGeneration.calcMotionCmds.followTrajectory.execute();
behaviorGeneration.calcMotionCmds.selector.desiredDirectionX2 = behaviorGeneration.calcMotionCmds.followTrajectory.desiredDirectionX;
behaviorGeneration.calcMotionCmds.selector.desiredDirectionY2 = behaviorGeneration.calcMotionCmds.followTrajectory.desiredDirectionY;
behaviorGeneration.calcMotionCmds.selector.signedDistanceToTrajectory2 = behaviorGeneration.calcMotionCmds.followTrajectory.signedDistanceToTrajectory;
behaviorGeneration.calcMotionCmds.d2v2.minVelocity = behaviorGeneration.calcMotionCmds.followTrajectory.minVelocity;
behaviorGeneration.calcMotionCmds.d2v2.maxVelocity = behaviorGeneration.calcMotionCmds.followTrajectory.maxVelocity;
behaviorGeneration.calcMotionCmds.d2v2.distance = behaviorGeneration.calcMotionCmds.followTrajectory.distance;
behaviorGeneration.calcMotionCmds.d2v1.execute();
behaviorGeneration.calcMotionCmds.selector.desiredVelocity1 = behaviorGeneration.calcMotionCmds.d2v1.velocity;
behaviorGeneration.calcMotionCmds.d2v2.execute();
behaviorGeneration.calcMotionCmds.selector.desiredVelocity2 = behaviorGeneration.calcMotionCmds.d2v2.velocity;
behaviorGeneration.calcMotionCmds.selector.execute();
motionPlanning.keepDirection.sab.v2x = behaviorGeneration.calcMotionCmds.selector.desiredDirectionX;
motionPlanning.keepDirection.sab.v2y = behaviorGeneration.calcMotionCmds.selector.desiredDirectionY;
motionPlanning.sac.signedDistanceToTrajectory = behaviorGeneration.calcMotionCmds.selector.signedDistanceToTrajectory;
motionPlanning.engineAndBrakes.pidParams.desiredVelocity = behaviorGeneration.calcMotionCmds.selector.desiredVelocity;
motionPlanning.engineAndBrakes.pidError.desiredVelocity = behaviorGeneration.calcMotionCmds.selector.desiredVelocity;
motionPlanning.keepDirection.sab.execute();
motionPlanning.sum1.t1 = motionPlanning.keepDirection.sab.angle;
motionPlanning.sac.execute();
motionPlanning.sum1.t2 = motionPlanning.sac.steeringAngleCorrection;
motionPlanning.engineAndBrakes.pidParams.execute();
motionPlanning.engineAndBrakes.pid.paramP = motionPlanning.engineAndBrakes.pidParams.paramP;
motionPlanning.engineAndBrakes.pid.paramI = motionPlanning.engineAndBrakes.pidParams.paramI;
motionPlanning.engineAndBrakes.pid.paramD = motionPlanning.engineAndBrakes.pidParams.paramD;
motionPlanning.engineAndBrakes.pid.paramDecayCoefficient = motionPlanning.engineAndBrakes.pidParams.paramDecayCoefficient;
motionPlanning.engineAndBrakes.pidError.execute();
motionPlanning.engineAndBrakes.abs1.input = motionPlanning.engineAndBrakes.pidError.error;
motionPlanning.engineAndBrakes.engineOrBrakes.error = motionPlanning.engineAndBrakes.pidError.error;
motionPlanning.sum1.execute();
motionPlanning.boundsSteering.eb.input = motionPlanning.sum1.result;
motionPlanning.engineAndBrakes.abs1.execute();
motionPlanning.engineAndBrakes.pid.error = motionPlanning.engineAndBrakes.abs1.output;
motionPlanning.engineAndBrakes.pid.execute();
motionPlanning.engineAndBrakes.engineOrBrakes.controlSignal = motionPlanning.engineAndBrakes.pid.control;
motionPlanning.boundsSteering.eb.execute();
steering = motionPlanning.boundsSteering.eb.output;
motionPlanning.engineAndBrakes.engineOrBrakes.execute();
motionPlanning.boundsEngine.eb.input = motionPlanning.engineAndBrakes.engineOrBrakes.engine;
motionPlanning.boundsBrakes.eb.input = motionPlanning.engineAndBrakes.engineOrBrakes.brakes;
motionPlanning.boundsBrakes.eb.execute();
brakes = motionPlanning.boundsBrakes.eb.output;
motionPlanning.boundsEngine.eb.execute();
engine = motionPlanning.boundsEngine.eb.output;
}

};
#endif
