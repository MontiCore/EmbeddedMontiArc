#ifndef SIMULATOR_MAINCONTROLLER
#define SIMULATOR_MAINCONTROLLER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "simulator_mainController_steerController1.h"
class simulator_mainController{
public:
double pathX[2];
double pathY[2];
double gpsX;
double gpsY;
double sensorSteering;
double sensorVelocity;
double sensorCompass;
double sensorWeather;
double sensorDistanceToLeft;
double sensorDistanceToRight;
double sensorCurrentSurface[3];
double minSteeringAngle;
double maxSteeringAngle;
double trajectoryError;
double brakesMinAcceleration;
double brakesMaxAcceleration;
double motorMinAcceleration;
double motorMaxAcceleration;
double maximumVelocity;
double wheelDistanceFrontBack;
double numberOfGears;
double deltaTime;
double actuatorEngine;
double actuatorBrake;
double actuatorGear;
double actuatorSteering;
double CONSTANTPORT1;
simulator_mainController_steerController1 steerController1;
void init()
{
this->CONSTANTPORT1 = 3.5;
steerController1.init();
}
void execute()
{
steerController1.x[0] = pathX[0];
steerController1.x[1] = pathX[1];
steerController1.y[0] = pathY[0];
steerController1.y[1] = pathY[1];
steerController1.gpsX = gpsX;
steerController1.gpsY = gpsY;
steerController1.orientation = sensorCompass;
steerController1.currentSteeringAngle = sensorSteering;
steerController1.minSteeringAngle = minSteeringAngle;
steerController1.maxSteeringAngle = maxSteeringAngle;
steerController1.execute();
actuatorEngine = CONSTANTPORT1;
actuatorSteering = steerController1.steeringAngle;
}

};
#endif
