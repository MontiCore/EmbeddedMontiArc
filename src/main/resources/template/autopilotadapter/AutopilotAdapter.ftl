<#include "/Common.ftl">
#ifndef AUTOPILOT_ADAPTER
#define AUTOPILOT_ADAPTER

#include "AutopilotAdapter.h"

#define _GLIBCXX_USE_CXX11_ABI 0

#include "./${viewModel.mainModelName}.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "./Helper.h"

${viewModel.mainModelName} AUTOPILOT_INSTANCE;



void copyJLongArrayToMatrix(JNIEnv *jenv, jlongArray &source, Matrix &dest) {
  jsize len = jenv -> GetArrayLength(source);
  if (len <= 0) {
    return;
  }
  jlong *body = jenv -> GetLongArrayElements(source, 0);
  for (int i=0; i<len; i++) {
    dest(0,i) = body[i];
  }
  jenv -> ReleaseLongArrayElements(source, body, 0);
}

void copyJDoubleArrayToMatrix(JNIEnv *jenv, jdoubleArray &source, Matrix &dest) {
  jsize len = jenv -> GetArrayLength(source);
  if (len <= 0) {
    return;
  }
  jdouble *body = jenv -> GetDoubleArrayElements(source, 0);
  for (int i=0; i<len; i++) {
    dest(0,i) = body[i];
  }
  jenv -> ReleaseDoubleArrayElements(source, body, 0);
}



/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_timeIncrement
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1timeIncrement
  (JNIEnv *jenv, jobject jobj, jdouble v) { AUTOPILOT_INSTANCE.timeIncrement = v; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_currentVelocity
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentVelocity
  (JNIEnv *jenv, jobject jobj, jdouble v) { AUTOPILOT_INSTANCE.currentVelocity = v; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_currentGpsLat
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1x
  (JNIEnv *jenv, jobject jobj, jdouble v) { AUTOPILOT_INSTANCE.x = v; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_currentGpsLon
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1y
  (JNIEnv *jenv, jobject jobj, jdouble v) { AUTOPILOT_INSTANCE.y = v; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_compass
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1compass
  (JNIEnv *jenv, jobject jobj, jdouble v) { AUTOPILOT_INSTANCE.compass = v; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_currentEngine
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentEngine
  (JNIEnv *jenv, jobject jobj, jdouble v) { AUTOPILOT_INSTANCE.currentEngine = v; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_currentSteering
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentSteering
  (JNIEnv *jenv, jobject jobj, jdouble v) { AUTOPILOT_INSTANCE.currentSteering = v; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_currentBrakes
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentBrakes
  (JNIEnv *jenv, jobject jobj, jdouble v) { AUTOPILOT_INSTANCE.currentBrakes = v; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_addNodes_length
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1trajectory_1length
  (JNIEnv *jenv, jobject jobj, jint v) { AUTOPILOT_INSTANCE.trajectory_length = v; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_addNodes_gpsLat
 * Signature: ([D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1trajectory_1x
  (JNIEnv *jenv, jobject jobj, jdoubleArray v) { copyJDoubleArrayToMatrix(jenv, v, AUTOPILOT_INSTANCE.trajectory_x); }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    set_addNodes_gpsLon
 * Signature: ([D)V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1trajectory_1y
  (JNIEnv *jenv, jobject jobj, jdoubleArray v) { copyJDoubleArrayToMatrix(jenv, v, AUTOPILOT_INSTANCE.trajectory_y); }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    get_engine
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_simulator_integration_AutopilotAdapter_get_1engine
  (JNIEnv *jenv, jobject jobj) { return AUTOPILOT_INSTANCE.engine; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    get_steering
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_simulator_integration_AutopilotAdapter_get_1steering
  (JNIEnv *jenv, jobject jobj) { return AUTOPILOT_INSTANCE.steering; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    get_brakes
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_simulator_integration_AutopilotAdapter_get_1brakes
  (JNIEnv *jenv, jobject jobj) { return AUTOPILOT_INSTANCE.brakes; }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    exec
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_exec
  (JNIEnv *jenv, jobject jobj) { AUTOPILOT_INSTANCE.execute(); }

/*
 * Class:     simulator_integration_AutopilotAdapter
 * Method:    init
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_init
  (JNIEnv *jenv, jobject jobj) {
    srand(time(NULL));
    Helper::init();
    AUTOPILOT_INSTANCE.init();
  }

#endif
