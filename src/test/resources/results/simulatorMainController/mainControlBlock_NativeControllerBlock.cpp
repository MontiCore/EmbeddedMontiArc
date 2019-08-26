/* (c) https://github.com/MontiCore/monticore */
#include "mainControlBlock_NativeControllerBlock.h"
#define _GLIBCXX_USE_CXX11_ABI 0
#include "simulator_mainController.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "Helper.h"
simulator_mainController mainController;
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_init
  (JNIEnv *jenv, jobject jobj){
      //TODO
      srand(time(NULL));
      Helper::init();
      mainController.init();
  }

JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_executeAllFunctionBlocks(JNIEnv* jenv, jobject jobj)
{
    // TODO
    //printf("Executed All Function Blocks");
    mainController.execute();
    return;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    getActuator_Brake
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Brake(JNIEnv* jenv, jobject jobj)
{
    return mainController.actuatorBrake;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    getActuator_Engine
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Engine(JNIEnv* jenv, jobject jobj)
{
    return mainController.actuatorEngine; 
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    getActuator_Gear
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Gear(JNIEnv* jenv, jobject jobj)
{
    return mainController.actuatorGear;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    getActuator_Steering
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Steering(JNIEnv* jenv, jobject jobj)
{
    return mainController.actuatorSteering;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Number_Of_Gears
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Number_1Of_1Gears(JNIEnv* jenv,
    jobject jobj,
    jint number_of_gears)
{
    mainController.numberOfGears = number_of_gears;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Wheelbase
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Wheelbase(JNIEnv* jenv,
    jobject jobj,
    jdouble distance)
{
    mainController.wheelDistanceFrontBack = distance;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Max_Total_Velocity
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Max_1Total_1Velocity(JNIEnv* jenv,
    jobject jobj,
    jdouble max_total_velocity)
{
    mainController.maximumVelocity = max_total_velocity;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Motor_Max_Acceleration
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Motor_1Max_1Acceleration(JNIEnv* jenv,
    jobject jobj,
    jdouble motor_max_acceleration)
{
    mainController.motorMaxAcceleration = motor_max_acceleration;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Motor_Min_Acceleration
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Motor_1Min_1Acceleration(JNIEnv* jenv,
    jobject jobj,
    jdouble motor_min_acceleration)
{
     mainController.motorMinAcceleration = motor_min_acceleration;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Brakes_Max_Acceleration
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Brakes_1Max_1Acceleration(JNIEnv* jenv,
    jobject jobj,
    jdouble brakes_max_acceleration)
{
    mainController.brakesMaxAcceleration = brakes_max_acceleration;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Brakes_Min_Acceleration
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Brakes_1Min_1Acceleration(JNIEnv* jenv,
    jobject jobj,
    jdouble brakes_min_acceleration)
{
    mainController.brakesMinAcceleration = brakes_min_acceleration;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Steering_Max_Angle
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Steering_1Max_1Angle(JNIEnv* jenv,
    jobject jobj,
    jdouble steering_max_angle)
{
    mainController.maxSteeringAngle = steering_max_angle;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Steering_Min_Angle
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Steering_1Min_1Angle(JNIEnv* jenv,
    jobject jobj,
    jdouble steering_min_angle)
{
    mainController.minSteeringAngle = steering_min_angle;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setConstant_Trajectory_Error
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Trajectory_1Error(JNIEnv* jenv,
    jobject jobj,
    jdouble trajectory_error)
{
    mainController.trajectoryError = trajectory_error;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setSensor_Velocity
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Velocity(JNIEnv* jenv,
    jobject jobj,
    jdouble sensor_velocity)
{
    mainController.sensorVelocity = sensor_velocity;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setSensor_Steering
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Steering(JNIEnv* jenv,
    jobject jobj,
    jdouble sensor_steering)
{
    mainController.sensorSteering = sensor_steering;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setSensor_Distance_To_Right
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Distance_1To_1Right(JNIEnv* jenv,
    jobject jobj,
    jdouble sensor_distance_to_right)
{
    mainController.sensorDistanceToRight = sensor_distance_to_right;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setSensor_Distance_To_Left
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Distance_1To_1Left(JNIEnv* jenv,
    jobject jobj,
    jdouble sensor_distance_to_left)
{
    mainController.sensorDistanceToLeft = sensor_distance_to_left;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setSensor_GPS_Coordinates
 * Signature: ([D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1GPS_1Coordinates(JNIEnv* jenv,
    jobject jobj,
    jdouble gpsX, jdouble gpsY)
{
    mainController.gpsX = gpsX;
    mainController.gpsY = gpsY;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setSensor_Current_Surface
 * Signature: ([D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Current_1Surface(JNIEnv* jenv,
    jobject jobj,
    jdouble sensor_current_surfaceA,jdouble sensor_current_surfaceB,jdouble sensor_current_surfaceC)
{
    mainController.sensorCurrentSurface[0] = sensor_current_surfaceA;
    mainController.sensorCurrentSurface[1] = sensor_current_surfaceB;
    mainController.sensorCurrentSurface[2] = sensor_current_surfaceC;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setSensor_Weather
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Weather(JNIEnv* jenv,
    jobject jobj,
    jdouble sensor_weather)
{
    mainController.sensorWeather = sensor_weather;
}

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setSensor_Compass
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Compass(JNIEnv* jenv,
    jobject jobj,
    jdouble sensor_compass)
{
    mainController.sensorCompass=sensor_compass;
}
/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setCurrentPathPoint
 * Signature: (DD)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setCurrentPathPoint
  (JNIEnv *, jobject jobj, jdouble curX, jdouble curY){
      mainController.pathX[0] = curX;
      mainController.pathY[0] = curY;
  }

/*
 * Class:     mainControlBlock_NativeControllerBlock
 * Method:    setNextPathPoint
 * Signature: (DD)V
 */
JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setNextPathPoint
  (JNIEnv *, jobject, jdouble nextX, jdouble nextY){
      mainController.pathX[1] = nextX;
      mainController.pathY[1] = nextY;
      
  }
