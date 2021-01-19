#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_c2cd.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot{
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
de_rwth_armin_modeling_autopilot_autopilot_c2cd c2cd;
de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration behaviorGeneration;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning motionPlanning;
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
std::thread thread1( [ this ] {this->c2cd.execute();});
std::thread thread2( [ this ] {this->behaviorGeneration.trimPath.execute();});
thread1.join();
thread2.join();
motionPlanning.keepDirection.sab.v1x = c2cd.currentDirectionX;
motionPlanning.keepDirection.sab.v1y = c2cd.currentDirectionY;
behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.trimmedTrajectoryLength = behaviorGeneration.trimPath.trimmedTrajectoryLength;
behaviorGeneration.calcMotionCmds.driveToFirstPosition.trimmedTrajectoryLength = behaviorGeneration.trimPath.trimmedTrajectoryLength;
behaviorGeneration.calcMotionCmds.followTrajectory.trimmedTrajectoryLength = behaviorGeneration.trimPath.trimmedTrajectoryLength;
behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.trimmedTrajectoryX = behaviorGeneration.trimPath.trimmedTrajectoryX;
behaviorGeneration.calcMotionCmds.driveToFirstPosition.trimmedTrajectoryX = behaviorGeneration.trimPath.trimmedTrajectoryX;
behaviorGeneration.calcMotionCmds.followTrajectory.trimmedTrajectoryX = behaviorGeneration.trimPath.trimmedTrajectoryX;
behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.trimmedTrajectoryY = behaviorGeneration.trimPath.trimmedTrajectoryY;
behaviorGeneration.calcMotionCmds.driveToFirstPosition.trimmedTrajectoryY = behaviorGeneration.trimPath.trimmedTrajectoryY;
behaviorGeneration.calcMotionCmds.followTrajectory.trimmedTrajectoryY = behaviorGeneration.trimPath.trimmedTrajectoryY;
std::thread thread3( [ this ] {this->behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.execute();});
thread3.join();
behaviorGeneration.calcMotionCmds.driveToFirstPosition.isDriveToFirstPosition = behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.result;
behaviorGeneration.calcMotionCmds.followTrajectory.isDriveToFirstPosition = behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.result;
behaviorGeneration.calcMotionCmds.selector.isDriveToFirstPosition = behaviorGeneration.calcMotionCmds.isDriveToFirstPosition.result;
std::thread thread4( [ this ] {this->behaviorGeneration.calcMotionCmds.driveToFirstPosition.execute();});
std::thread thread5( [ this ] {this->behaviorGeneration.calcMotionCmds.followTrajectory.execute();});
thread4.join();
thread5.join();
behaviorGeneration.calcMotionCmds.selector.desiredDirectionX1 = behaviorGeneration.calcMotionCmds.driveToFirstPosition.desiredDirectionX;
behaviorGeneration.calcMotionCmds.selector.desiredDirectionY1 = behaviorGeneration.calcMotionCmds.driveToFirstPosition.desiredDirectionY;
behaviorGeneration.calcMotionCmds.selector.signedDistanceToTrajectory1 = behaviorGeneration.calcMotionCmds.driveToFirstPosition.signedDistanceToTrajectory;
behaviorGeneration.calcMotionCmds.d2v1.minVelocity = behaviorGeneration.calcMotionCmds.driveToFirstPosition.minVelocity;
behaviorGeneration.calcMotionCmds.d2v1.maxVelocity = behaviorGeneration.calcMotionCmds.driveToFirstPosition.maxVelocity;
behaviorGeneration.calcMotionCmds.d2v1.distance = behaviorGeneration.calcMotionCmds.driveToFirstPosition.distance;
behaviorGeneration.calcMotionCmds.selector.desiredDirectionX2 = behaviorGeneration.calcMotionCmds.followTrajectory.desiredDirectionX;
behaviorGeneration.calcMotionCmds.selector.desiredDirectionY2 = behaviorGeneration.calcMotionCmds.followTrajectory.desiredDirectionY;
behaviorGeneration.calcMotionCmds.selector.signedDistanceToTrajectory2 = behaviorGeneration.calcMotionCmds.followTrajectory.signedDistanceToTrajectory;
behaviorGeneration.calcMotionCmds.d2v2.minVelocity = behaviorGeneration.calcMotionCmds.followTrajectory.minVelocity;
behaviorGeneration.calcMotionCmds.d2v2.maxVelocity = behaviorGeneration.calcMotionCmds.followTrajectory.maxVelocity;
behaviorGeneration.calcMotionCmds.d2v2.distance = behaviorGeneration.calcMotionCmds.followTrajectory.distance;
std::thread thread6( [ this ] {this->behaviorGeneration.calcMotionCmds.d2v1.execute();});
std::thread thread7( [ this ] {this->behaviorGeneration.calcMotionCmds.d2v2.execute();});
thread6.join();
thread7.join();
behaviorGeneration.calcMotionCmds.selector.desiredVelocity1 = behaviorGeneration.calcMotionCmds.d2v1.velocity;
behaviorGeneration.calcMotionCmds.selector.desiredVelocity2 = behaviorGeneration.calcMotionCmds.d2v2.velocity;
std::thread thread8( [ this ] {this->behaviorGeneration.calcMotionCmds.selector.execute();});
thread8.join();
motionPlanning.keepDirection.sab.v2x = behaviorGeneration.calcMotionCmds.selector.desiredDirectionX;
motionPlanning.keepDirection.sab.v2y = behaviorGeneration.calcMotionCmds.selector.desiredDirectionY;
motionPlanning.sac.signedDistanceToTrajectory = behaviorGeneration.calcMotionCmds.selector.signedDistanceToTrajectory;
motionPlanning.engineAndBrakes.pidParams.desiredVelocity = behaviorGeneration.calcMotionCmds.selector.desiredVelocity;
motionPlanning.engineAndBrakes.pidError.desiredVelocity = behaviorGeneration.calcMotionCmds.selector.desiredVelocity;
std::thread thread9( [ this ] {this->motionPlanning.keepDirection.sab.execute();});
std::thread thread10( [ this ] {this->motionPlanning.sac.execute();});
std::thread thread11( [ this ] {this->motionPlanning.engineAndBrakes.pidParams.execute();});
std::thread thread12( [ this ] {this->motionPlanning.engineAndBrakes.pidError.execute();});
thread9.join();
thread10.join();
thread11.join();
thread12.join();
motionPlanning.sum1.t1 = motionPlanning.keepDirection.sab.angle;
motionPlanning.sum1.t2 = motionPlanning.sac.steeringAngleCorrection;
motionPlanning.engineAndBrakes.pid.paramP = motionPlanning.engineAndBrakes.pidParams.paramP;
motionPlanning.engineAndBrakes.pid.paramI = motionPlanning.engineAndBrakes.pidParams.paramI;
motionPlanning.engineAndBrakes.pid.paramD = motionPlanning.engineAndBrakes.pidParams.paramD;
motionPlanning.engineAndBrakes.pid.paramDecayCoefficient = motionPlanning.engineAndBrakes.pidParams.paramDecayCoefficient;
motionPlanning.engineAndBrakes.abs1.input = motionPlanning.engineAndBrakes.pidError.error;
motionPlanning.engineAndBrakes.engineOrBrakes.error = motionPlanning.engineAndBrakes.pidError.error;
std::thread thread13( [ this ] {this->motionPlanning.sum1.execute();});
std::thread thread14( [ this ] {this->motionPlanning.engineAndBrakes.abs1.execute();});
thread13.join();
thread14.join();
motionPlanning.boundsSteering.eb.input = motionPlanning.sum1.result;
motionPlanning.engineAndBrakes.pid.error = motionPlanning.engineAndBrakes.abs1.output;
std::thread thread15( [ this ] {this->motionPlanning.engineAndBrakes.pid.execute();});
std::thread thread16( [ this ] {this->motionPlanning.boundsSteering.eb.execute();});
thread15.join();
thread16.join();
motionPlanning.engineAndBrakes.engineOrBrakes.controlSignal = motionPlanning.engineAndBrakes.pid.control;
steering = motionPlanning.boundsSteering.eb.output;
std::thread thread17( [ this ] {this->motionPlanning.engineAndBrakes.engineOrBrakes.execute();});
thread17.join();
motionPlanning.boundsEngine.eb.input = motionPlanning.engineAndBrakes.engineOrBrakes.engine;
motionPlanning.boundsBrakes.eb.input = motionPlanning.engineAndBrakes.engineOrBrakes.brakes;
std::thread thread18( [ this ] {this->motionPlanning.boundsBrakes.eb.execute();});
std::thread thread19( [ this ] {this->motionPlanning.boundsEngine.eb.execute();});
thread18.join();
thread19.join();
brakes = motionPlanning.boundsBrakes.eb.output;
engine = motionPlanning.boundsEngine.eb.output;
}

};
#endif
