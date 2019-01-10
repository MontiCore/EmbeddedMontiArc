#include "jni/simulator_integration_AutopilotAdapter.h"

#include "interfaces/autopilot_interface.h"


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
    AutopilotInterface::instance.emulator.set_input_double( AutopilotInterface::SET_TIME_INC, time_inc );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_currentVelocity
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentVelocity
( JNIEnv *, jobject, jdouble vel ) {
    AutopilotInterface::instance.emulator.set_input_double( AutopilotInterface::SET_VELOCITY, vel );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_x
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1x
( JNIEnv *, jobject, jdouble x ) {
    AutopilotInterface::instance.emulator.set_input_double( AutopilotInterface::SET_X, x );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_y
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1y
( JNIEnv *, jobject, jdouble y ) {
    AutopilotInterface::instance.emulator.set_input_double( AutopilotInterface::SET_Y, y );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_compass
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1compass
( JNIEnv *, jobject, jdouble compass_value ) {
    AutopilotInterface::instance.emulator.set_input_double( AutopilotInterface::SET_COMPASS, compass_value );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_currentEngine
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentEngine
( JNIEnv *, jobject, jdouble engine_value ) {
    AutopilotInterface::instance.emulator.set_input_double( AutopilotInterface::SET_ENGINE, engine_value );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_currentSteering
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentSteering
( JNIEnv *, jobject, jdouble steering_value ) {
    AutopilotInterface::instance.emulator.set_input_double( AutopilotInterface::SET_STEERING, steering_value );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_currentBrakes
* Signature: (D)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1currentBrakes
( JNIEnv *, jobject, jdouble brake_val ) {
    AutopilotInterface::instance.emulator.set_input_double( AutopilotInterface::SET_BRAKES, brake_val );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    set_trajectory_length
* Signature: (I)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_set_1trajectory_1length
( JNIEnv *, jobject, jint length ) {
    AutopilotInterface::instance.emulator.set_input_int( AutopilotInterface::SET_X, length );
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
    AutopilotInterface::instance.emulator.set_input_array( AutopilotInterface::SET_TRAJECTORY_X, data, count );
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
    AutopilotInterface::instance.emulator.set_input_array( AutopilotInterface::SET_TRAJECTORY_Y, data, count );
    free_double_array( jenv, darray, data );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    get_engine
* Signature: ()D
*/
JNIEXPORT jdouble JNICALL Java_simulator_integration_AutopilotAdapter_get_1engine
( JNIEnv *, jobject ) {
    return AutopilotInterface::instance.emulator.get_output( AutopilotInterface::GET_ENGINE );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    get_steering
* Signature: ()D
*/
JNIEXPORT jdouble JNICALL Java_simulator_integration_AutopilotAdapter_get_1steering
( JNIEnv *, jobject ) {
    return AutopilotInterface::instance.emulator.get_output( AutopilotInterface::GET_STEERING );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    get_brakes
* Signature: ()D
*/
JNIEXPORT jdouble JNICALL Java_simulator_integration_AutopilotAdapter_get_1brakes
( JNIEnv *, jobject ) {
    return AutopilotInterface::instance.emulator.get_output( AutopilotInterface::GET_BRAKES );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    exec
* Signature: ()V
*/
JNIEXPORT void JNICALL Java_simulator_integration_AutopilotAdapter_exec
( JNIEnv *, jobject ) {
    AutopilotInterface::instance.exec();
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
    AutopilotInterface::instance.emulator.call( AutopilotInterface::INIT );
}

/*
* Class:     simulator_integration_AutopilotAdapter
* Method:    message
* Signature: (Ljava/lang/String;)Ljava/lang/String;
*/
JNIEXPORT jstring JNICALL Java_simulator_integration_AutopilotAdapter_message
( JNIEnv *jni, jobject, jstring string ) {
    auto message = jni->GetStringUTFChars( string, 0 );
    auto res = AutopilotInterface::instance.message( message );
    jni->ReleaseStringUTFChars( string, message );
    return jni->NewStringUTF( res.c_str() );
}