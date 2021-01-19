/* (c) https://github.com/MontiCore/monticore */
#ifndef SIMULATOR_MAINCONTROLLER
#define SIMULATOR_MAINCONTROLLER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "simulator_mainController_steerController1.h"
class simulator_mainController{
public:
double CONSTANTPORT1;
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
int numberOfGears;
double deltaTime;
double actuatorEngine;
double actuatorBrake;
int actuatorGear;
double actuatorSteering;
simulator_mainController_steerController1 steerController1;
void init()
{
this->CONSTANTPORT1 = 3.5;
steerController1.init();
}
void execute()
{
steerController1.steeringCalculator.x[0] = pathX[0];
steerController1.steeringCalculator.x[1] = pathX[1];
steerController1.steeringCalculator.y[0] = pathY[0];
steerController1.steeringCalculator.y[1] = pathY[1];
steerController1.steeringCalculator.gpsX = gpsX;
steerController1.steeringCalculator.gpsY = gpsY;
steerController1.steeringCalculator.currentSteeringAngle = sensorSteering;
steerController1.steeringCalculator.orientation = sensorCompass;
steerController1.steeringCalculator.minSteeringAngle = minSteeringAngle;
steerController1.steeringCalculator.maxSteeringAngle = maxSteeringAngle;
actuatorEngine = 3.5;
steerController1.steeringCalculator.execute();
actuatorSteering = steerController1.steeringCalculator.newSteeringAngle;
}

};
#endif
