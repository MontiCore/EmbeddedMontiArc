#include "jni/simulator_integration_AutopilotAdapter.h"

#include "autopilot_interface.h"


void get_double_array( JNIEnv *jenv, jdoubleArray &source, double *&data, uint &count ) {
    count = jenv->GetArrayLength( source );
    if ( count <= 0 )
        return;
    data = jenv->GetDoubleArrayElements( source, 0 );
}

void free_double_array( JNIEnv *jenv, jdoubleArray &source, double *&data ) {
    jenv->ReleaseDoubleArrayElements( source, data, 0 );
    data = nullptr;
}






/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_timeIncrement
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1timeIncrement
( JNIEnv *, jobject, jdouble time_inc ) {
    AutopilotInterface::instance.jni_set_timeIncrement( time_inc );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_currentVelocity
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentVelocity
( JNIEnv *, jobject, jdouble vel ) {
    AutopilotInterface::instance.jni_set_currentVelocity( vel );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_x
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1x
( JNIEnv *, jobject, jdouble x ) {
    AutopilotInterface::instance.jni_set_x( x );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_y
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1y
( JNIEnv *, jobject, jdouble y ) {
    AutopilotInterface::instance.jni_set_y( y );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_compass
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1compass
( JNIEnv *, jobject, jdouble compass_value ) {
    AutopilotInterface::instance.jni_set_compass( compass_value );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_currentEngine
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentEngine
( JNIEnv *, jobject, jdouble engine_value ) {
    AutopilotInterface::instance.jni_set_currentEngine( engine_value );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_currentSteering
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentSteering
( JNIEnv *, jobject, jdouble steering_value ) {
    AutopilotInterface::instance.jni_set_currentSteering( steering_value );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_currentBrakes
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentBrakes
( JNIEnv *, jobject, jdouble brake_val ) {
    AutopilotInterface::instance.jni_set_currentBrakes( brake_val );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_trajectory_length
* Signature: (I)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1trajectory_1length
( JNIEnv *, jobject, jint length ) {
    AutopilotInterface::instance.jni_set_trajectory_length( length );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_trajectory_x
* Signature: ([D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1trajectory_1x
( JNIEnv *jenv, jobject, jdoubleArray darray ) {
    double *data;
    uint count;
    get_double_array( jenv, darray, data, count );
    AutopilotInterface::instance.jni_set_trajectory_x( data, count );
    free_double_array( jenv, darray, data );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_trajectory_y
* Signature: ([D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1trajectory_1y
( JNIEnv *jenv, jobject, jdoubleArray darray ) {
    double *data;
    uint count;
    get_double_array( jenv, darray, data, count );
    AutopilotInterface::instance.jni_set_trajectory_y( data, count );
    free_double_array( jenv, darray, data );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    get_engine
* Signature: ()D
*/
JNIEXPORT jdouble JNICALL Java_simulator_integration_AutopilotAdapter_get_1engine
( JNIEnv *, jobject ) {
    return AutopilotInterface::instance.jni_get_engine();
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    get_steering
* Signature: ()D
*/
JNIEXPORT jdouble JNICALL Java_simulator_integration_AutopilotAdapter_get_1steering
( JNIEnv *, jobject ) {
    return AutopilotInterface::instance.jni_get_steering();
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    get_brakes
* Signature: ()D
*/
JNIEXPORT jdouble JNICALL Java_simulator_integration_AutopilotAdapter_get_1brakes
( JNIEnv *, jobject ) {
    return AutopilotInterface::instance.jni_get_brakes();
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    exec
* Signature: ()V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_exec
( JNIEnv *, jobject ) {
    AutopilotInterface::instance.jni_exec();
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    init
* Signature: ()V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_init
( JNIEnv *, jobject ) {
    if ( !AutopilotInterface::instance.loaded )
        AutopilotInterface::instance.init();
    AutopilotInterface::instance.jni_init();
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    message
* Signature: (Ljava/lang/String;)Ljava/lang/String;
*/
JNIEXPORT jstring JNICALL Java_simulator_integration_AutopilotAdapter_message
( JNIEnv *, jobject, jstring ) {

    return jstring();
}