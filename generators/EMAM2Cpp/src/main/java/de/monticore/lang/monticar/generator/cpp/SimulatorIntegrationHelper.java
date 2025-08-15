/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.FileContent;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class SimulatorIntegrationHelper {
    public static String fileName = "mainControlBlock_NativeControllerBlock";
    public static List<FileContent> getSimulatorIntegrationHelperFileContent() {

        FileContent fileContent = new FileContent();
        fileContent.setFileName(fileName + ".cpp");
        String fileContentString = "#include \"mainControlBlock_NativeControllerBlock.h\"\n" +
                "#define _GLIBCXX_USE_CXX11_ABI 0\n" +
                "#include \"simulator_mainController.h\"\n" +
                "#include <stdlib.h>\n" +
                "#include <stdio.h>\n" +
                "#include <time.h>\n" +
                "#include \"Helper.h\"\n" +
                "simulator_mainController mainController;\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_init\n" +
                "  (JNIEnv *jenv, jobject jobj){\n" +
                "      //TODO\n" +
                "      srand(time(NULL));\n" +
                "      Helper::init();\n" +
                "      mainController.init();\n" +
                "  }\n" +
                "\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_executeAllFunctionBlocks(JNIEnv* jenv, jobject jobj)\n" +
                "{\n" +
                "    // TODO\n" +
                "    //printf(\"Executed All Function Blocks\");\n" +
                "    mainController.execute();\n" +
                "    return;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    getActuator_Brake\n" +
                " * Signature: ()D\n" +
                " */\n" +
                "JNIEXPORT jdouble JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Brake(JNIEnv* jenv, jobject jobj)\n" +
                "{\n" +
                "    return mainController.actuatorBrake;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    getActuator_Engine\n" +
                " * Signature: ()D\n" +
                " */\n" +
                "JNIEXPORT jdouble JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Engine(JNIEnv* jenv, jobject jobj)\n" +
                "{\n" +
                "    return mainController.actuatorEngine; \n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    getActuator_Gear\n" +
                " * Signature: ()I\n" +
                " */\n" +
                "JNIEXPORT jint JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Gear(JNIEnv* jenv, jobject jobj)\n" +
                "{\n" +
                "    return mainController.actuatorGear;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    getActuator_Steering\n" +
                " * Signature: ()D\n" +
                " */\n" +
                "JNIEXPORT jdouble JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Steering(JNIEnv* jenv, jobject jobj)\n" +
                "{\n" +
                "    return mainController.actuatorSteering;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Number_Of_Gears\n" +
                " * Signature: (I)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Number_1Of_1Gears(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jint number_of_gears)\n" +
                "{\n" +
                "    mainController.numberOfGears = number_of_gears;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Wheelbase\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Wheelbase(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble distance)\n" +
                "{\n" +
                "    mainController.wheelDistanceFrontBack = distance;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Max_Total_Velocity\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Max_1Total_1Velocity(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble max_total_velocity)\n" +
                "{\n" +
                "    mainController.maximumVelocity = max_total_velocity;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Motor_Max_Acceleration\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Motor_1Max_1Acceleration(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble motor_max_acceleration)\n" +
                "{\n" +
                "    mainController.motorMaxAcceleration = motor_max_acceleration;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Motor_Min_Acceleration\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Motor_1Min_1Acceleration(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble motor_min_acceleration)\n" +
                "{\n" +
                "     mainController.motorMinAcceleration = motor_min_acceleration;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Brakes_Max_Acceleration\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Brakes_1Max_1Acceleration(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble brakes_max_acceleration)\n" +
                "{\n" +
                "    mainController.brakesMaxAcceleration = brakes_max_acceleration;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Brakes_Min_Acceleration\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Brakes_1Min_1Acceleration(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble brakes_min_acceleration)\n" +
                "{\n" +
                "    mainController.brakesMinAcceleration = brakes_min_acceleration;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Steering_Max_Angle\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Steering_1Max_1Angle(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble steering_max_angle)\n" +
                "{\n" +
                "    mainController.maxSteeringAngle = steering_max_angle;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Steering_Min_Angle\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Steering_1Min_1Angle(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble steering_min_angle)\n" +
                "{\n" +
                "    mainController.minSteeringAngle = steering_min_angle;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Trajectory_Error\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Trajectory_1Error(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble trajectory_error)\n" +
                "{\n" +
                "    mainController.trajectoryError = trajectory_error;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Velocity\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Velocity(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble sensor_velocity)\n" +
                "{\n" +
                "    mainController.sensorVelocity = sensor_velocity;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Steering\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Steering(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble sensor_steering)\n" +
                "{\n" +
                "    mainController.sensorSteering = sensor_steering;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Distance_To_Right\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Distance_1To_1Right(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble sensor_distance_to_right)\n" +
                "{\n" +
                "    mainController.sensorDistanceToRight = sensor_distance_to_right;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Distance_To_Left\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Distance_1To_1Left(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble sensor_distance_to_left)\n" +
                "{\n" +
                "    mainController.sensorDistanceToLeft = sensor_distance_to_left;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_GPS_Coordinates\n" +
                " * Signature: ([D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1GPS_1Coordinates(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble gpsX, jdouble gpsY)\n" +
                "{\n" +
                "    mainController.gpsX = gpsX;\n" +
                "    mainController.gpsY = gpsY;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Current_Surface\n" +
                " * Signature: ([D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Current_1Surface(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble sensor_current_surfaceA,jdouble sensor_current_surfaceB,jdouble sensor_current_surfaceC)\n" +
                "{\n" +
                "    mainController.sensorCurrentSurface[0] = sensor_current_surfaceA;\n" +
                "    mainController.sensorCurrentSurface[1] = sensor_current_surfaceB;\n" +
                "    mainController.sensorCurrentSurface[2] = sensor_current_surfaceC;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Weather\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Weather(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble sensor_weather)\n" +
                "{\n" +
                "    mainController.sensorWeather = sensor_weather;\n" +
                "}\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Compass\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Compass(JNIEnv* jenv,\n" +
                "    jobject jobj,\n" +
                "    jdouble sensor_compass)\n" +
                "{\n" +
                "    mainController.sensorCompass=sensor_compass;\n" +
                "}\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setCurrentPathPoint\n" +
                " * Signature: (DD)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setCurrentPathPoint\n" +
                "  (JNIEnv *, jobject jobj, jdouble curX, jdouble curY){\n" +
                "      mainController.pathX[0] = curX;\n" +
                "      mainController.pathY[0] = curY;\n" +
                "  }\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setNextPathPoint\n" +
                " * Signature: (DD)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setNextPathPoint\n" +
                "  (JNIEnv *, jobject, jdouble nextX, jdouble nextY){\n" +
                "      mainController.pathX[1] = nextX;\n" +
                "      mainController.pathY[1] = nextY;\n" +
                "      \n" +
                "  }";

        fileContent.setFileContent(fileContentString);

        FileContent headerContent=new FileContent();
        headerContent.setFileName(fileName +".h");
        String headerContentString="/* DO NOT EDIT THIS FILE - it is machine generated */\n" +
                "#include <jni.h>\n" +
                "/* Header for class mainControlBlock_NativeControllerBlock */\n" +
                "\n" +
                "#ifndef _Included_mainControlBlock_NativeControllerBlock\n" +
                "#define _Included_mainControlBlock_NativeControllerBlock\n" +
                "#ifdef __cplusplus\n" +
                "extern \"C\" {\n" +
                "#endif\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    init\n" +
                " * Signature: ()V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_init\n" +
                "  (JNIEnv *, jobject);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    executeAllFunctionBlocks\n" +
                " * Signature: ()V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_executeAllFunctionBlocks\n" +
                "  (JNIEnv *, jobject);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    getActuator_Brake\n" +
                " * Signature: ()D\n" +
                " */\n" +
                "JNIEXPORT jdouble JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Brake\n" +
                "  (JNIEnv *, jobject);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    getActuator_Engine\n" +
                " * Signature: ()D\n" +
                " */\n" +
                "JNIEXPORT jdouble JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Engine\n" +
                "  (JNIEnv *, jobject);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    getActuator_Gear\n" +
                " * Signature: ()I\n" +
                " */\n" +
                "JNIEXPORT jint JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Gear\n" +
                "  (JNIEnv *, jobject);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    getActuator_Steering\n" +
                " * Signature: ()D\n" +
                " */\n" +
                "JNIEXPORT jdouble JNICALL Java_mainControlBlock_NativeControllerBlock_getActuator_1Steering\n" +
                "  (JNIEnv *, jobject);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Number_Of_Gears\n" +
                " * Signature: (I)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Number_1Of_1Gears\n" +
                "  (JNIEnv *, jobject, jint);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Wheelbase\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Wheelbase\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Max_Total_Velocity\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Max_1Total_1Velocity\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Motor_Max_Acceleration\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Motor_1Max_1Acceleration\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Motor_Min_Acceleration\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Motor_1Min_1Acceleration\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Brakes_Max_Acceleration\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Brakes_1Max_1Acceleration\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Brakes_Min_Acceleration\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Brakes_1Min_1Acceleration\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Steering_Max_Angle\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Steering_1Max_1Angle\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Steering_Min_Angle\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Steering_1Min_1Angle\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setConstant_Trajectory_Error\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setConstant_1Trajectory_1Error\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setCurrentPathPoint\n" +
                " * Signature: (DD)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setCurrentPathPoint\n" +
                "  (JNIEnv *, jobject, jdouble, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setNextPathPoint\n" +
                " * Signature: (DD)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setNextPathPoint\n" +
                "  (JNIEnv *, jobject, jdouble, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Velocity\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Velocity\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Steering\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Steering\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Distance_To_Right\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Distance_1To_1Right\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Distance_To_Left\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Distance_1To_1Left\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_GPS_Coordinates\n" +
                " * Signature: ([D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1GPS_1Coordinates\n" +
                "  (JNIEnv *, jobject, jdouble, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Current_Surface\n" +
                " * Signature: ([D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Current_1Surface\n" +
                "  (JNIEnv *, jobject, jdouble, jdouble, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Weather\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Weather\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "/*\n" +
                " * Class:     mainControlBlock_NativeControllerBlock\n" +
                " * Method:    setSensor_Compass\n" +
                " * Signature: (D)V\n" +
                " */\n" +
                "JNIEXPORT void JNICALL Java_mainControlBlock_NativeControllerBlock_setSensor_1Compass\n" +
                "  (JNIEnv *, jobject, jdouble);\n" +
                "\n" +
                "#ifdef __cplusplus\n" +
                "}\n" +
                "#endif\n" +
                "#endif\n";

        headerContent.setFileContent(headerContentString);
        List<FileContent> fileContents=new ArrayList<>();
        fileContents.add(fileContent);
        fileContents.add(headerContent);
        return fileContents;
    }
}
