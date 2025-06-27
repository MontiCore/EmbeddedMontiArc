/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE9_MYCOMPONENT9
#define TESTING_SUBPACKAGE9_MYCOMPONENT9
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage9_myComponent9_trimPath.h"
#include "testing_subpackage9_myComponent9_calcMotionCmds.h"
class testing_subpackage9_myComponent9{
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
testing_subpackage9_myComponent9_trimPath trimPath;
testing_subpackage9_myComponent9_calcMotionCmds calcMotionCmds;
void init()
{
plannedTrajectoryX=Matrix(1,100);
plannedTrajectoryY=Matrix(1,100);
trimPath.init();
calcMotionCmds.init();
}
void execute()
{
trimPath.currentPositionX = currentPositionX;
calcMotionCmds.isDriveToFirstPosition.currentPositionX = currentPositionX;
calcMotionCmds.driveToFirstPosition.currentPositionX = currentPositionX;
calcMotionCmds.followTrajectory.currentPositionX = currentPositionX;
trimPath.currentPositionY = currentPositionY;
calcMotionCmds.isDriveToFirstPosition.currentPositionY = currentPositionY;
calcMotionCmds.driveToFirstPosition.currentPositionY = currentPositionY;
calcMotionCmds.followTrajectory.currentPositionY = currentPositionY;
trimPath.plannedTrajectoryLength = plannedTrajectoryLength;
trimPath.plannedTrajectoryX = plannedTrajectoryX;
trimPath.plannedTrajectoryY = plannedTrajectoryY;
trimPath.execute();
calcMotionCmds.isDriveToFirstPosition.trimmedTrajectoryLength = trimPath.trimmedTrajectoryLength;
calcMotionCmds.driveToFirstPosition.trimmedTrajectoryLength = trimPath.trimmedTrajectoryLength;
calcMotionCmds.followTrajectory.trimmedTrajectoryLength = trimPath.trimmedTrajectoryLength;
calcMotionCmds.isDriveToFirstPosition.trimmedTrajectoryX = trimPath.trimmedTrajectoryX;
calcMotionCmds.driveToFirstPosition.trimmedTrajectoryX = trimPath.trimmedTrajectoryX;
calcMotionCmds.followTrajectory.trimmedTrajectoryX = trimPath.trimmedTrajectoryX;
calcMotionCmds.isDriveToFirstPosition.trimmedTrajectoryY = trimPath.trimmedTrajectoryY;
calcMotionCmds.driveToFirstPosition.trimmedTrajectoryY = trimPath.trimmedTrajectoryY;
calcMotionCmds.followTrajectory.trimmedTrajectoryY = trimPath.trimmedTrajectoryY;
calcMotionCmds.isDriveToFirstPosition.execute();
calcMotionCmds.driveToFirstPosition.isDriveToFirstPosition = calcMotionCmds.isDriveToFirstPosition.result;
calcMotionCmds.followTrajectory.isDriveToFirstPosition = calcMotionCmds.isDriveToFirstPosition.result;
calcMotionCmds.selector.isDriveToFirstPosition = calcMotionCmds.isDriveToFirstPosition.result;
calcMotionCmds.driveToFirstPosition.execute();
calcMotionCmds.selector.desiredDirectionX1 = calcMotionCmds.driveToFirstPosition.desiredDirectionX;
calcMotionCmds.selector.desiredDirectionY1 = calcMotionCmds.driveToFirstPosition.desiredDirectionY;
calcMotionCmds.selector.signedDistanceToTrajectory1 = calcMotionCmds.driveToFirstPosition.signedDistanceToTrajectory;
calcMotionCmds.d2v1.minVelocity = calcMotionCmds.driveToFirstPosition.minVelocity;
calcMotionCmds.d2v1.maxVelocity = calcMotionCmds.driveToFirstPosition.maxVelocity;
calcMotionCmds.d2v1.distance = calcMotionCmds.driveToFirstPosition.distance;
calcMotionCmds.followTrajectory.execute();
calcMotionCmds.selector.desiredDirectionX2 = calcMotionCmds.followTrajectory.desiredDirectionX;
calcMotionCmds.selector.desiredDirectionY2 = calcMotionCmds.followTrajectory.desiredDirectionY;
calcMotionCmds.selector.signedDistanceToTrajectory2 = calcMotionCmds.followTrajectory.signedDistanceToTrajectory;
calcMotionCmds.d2v2.minVelocity = calcMotionCmds.followTrajectory.minVelocity;
calcMotionCmds.d2v2.maxVelocity = calcMotionCmds.followTrajectory.maxVelocity;
calcMotionCmds.d2v2.distance = calcMotionCmds.followTrajectory.distance;
calcMotionCmds.d2v1.execute();
calcMotionCmds.selector.desiredVelocity1 = calcMotionCmds.d2v1.velocity;
calcMotionCmds.d2v2.execute();
calcMotionCmds.selector.desiredVelocity2 = calcMotionCmds.d2v2.velocity;
calcMotionCmds.selector.execute();
desiredDirectionX = calcMotionCmds.selector.desiredDirectionX;
desiredDirectionY = calcMotionCmds.selector.desiredDirectionY;
signedDistanceToTrajectory = calcMotionCmds.selector.signedDistanceToTrajectory;
desiredVelocity = calcMotionCmds.selector.desiredVelocity;
}

};
#endif
