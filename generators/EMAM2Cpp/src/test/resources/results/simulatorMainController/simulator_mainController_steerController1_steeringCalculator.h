/* (c) https://github.com/MontiCore/monticore */
#ifndef SIMULATOR_MAINCONTROLLER_STEERCONTROLLER1_STEERINGCALCULATOR
#define SIMULATOR_MAINCONTROLLER_STEERCONTROLLER1_STEERINGCALCULATOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class simulator_mainController_steerController1_steeringCalculator{
public:
double x[2];
double y[2];
double gpsX;
double gpsY;
double orientation;
double currentSteeringAngle;
double minSteeringAngle;
double maxSteeringAngle;
double newSteeringAngle;
void init()
{
}
void execute()
{
double distance;
double res = (y[2-1]-y[1-1])*gpsX;
res -= (x[2-1]-x[1-1])*gpsY;
res += x[2-1]*y[1-1];
res -= y[2-1]*x[1-1];
double xDiff = x[1-1]-x[2-1];
double yDiff = y[1-1]-y[2-1];
res /= (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(xDiff*xDiff+yDiff*yDiff),1)));
distance = res;
double globalOrientation = orientation*(M_PI/180);
if(((globalOrientation > M_PI))){
globalOrientation -= 2*M_PI;
}
double orientedDistance = distance;
double angleTowardsTrajectory = (Helper::getDoubleFromOctaveListFirstResult(Fatan(Helper::convertToOctaveValueList(orientedDistance/2),1)));
double orientationOfTrajectory;
double v1 = x[2-1]-x[1-1];
double v2 = y[2-1]-y[1-1];
double cosineAngle = v2/(Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(v1*v1+v2*v2),1)));
double angle = (Helper::getDoubleFromOctaveListFirstResult(Facos(Helper::convertToOctaveValueList(cosineAngle),1)));
if(((v1 > 0))){
orientationOfTrajectory = -angle;
}
else {
orientationOfTrajectory = angle;
}
double angleTrajectoryAndCarDirection = orientationOfTrajectory-globalOrientation;
double finalAngle = angleTrajectoryAndCarDirection+angleTowardsTrajectory;
if(((finalAngle > M_PI))){
finalAngle -= 2*M_PI;
}
else if(((finalAngle < -M_PI))){
finalAngle += 2*M_PI;
}
newSteeringAngle = finalAngle*(180/M_PI);
angle = newSteeringAngle;
if((angle < minSteeringAngle)){
angle = minSteeringAngle;
}
else if((angle > maxSteeringAngle)){
angle = maxSteeringAngle;
}
newSteeringAngle = -angle;
}

};
#endif
