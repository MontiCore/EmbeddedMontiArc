#include "emulation/simulator_integration_HardwareEmulatorInterface.h"

#include "autopilot/autopilot_interface.h"
#include "autopilot/autopilot_functions.h"



bool str_equals( const char *a, const char *b ) {
    uint i = 0;
    while ( a[i] && b[i] ) {
        if ( a[i] != b[i] )
            return false;
        ++i;
    }
    return a[i] == b[i];
}

int get_autopilot_function_id( const char *name ) {
    int id = 0;
    for ( auto &af : AutopilotFunction::autopilot_inputs ) {
        if ( str_equals( name, af.name ) )
            return id;
        ++id;
    }
    return -1;
}

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

//outputs.put("engine", get_engine());
//outputs.put("steering", get_steering());
//outputs.put("brakes", get_brakes());

/*
* Class:     simulator_integration_HardwareEmulatorInterface
* Method:    init
* Signature: ()Z
*/
JNIEXPORT jboolean JNICALL Java_simulator_integration_HardwareEmulatorInterface_init
( JNIEnv *, jobject ) {
    return AutopilotInterface::instance.init();
}

/*
* Class:     simulator_integration_HardwareEmulatorInterface
* Method:    add_input
* Signature: (Ljava/lang/String;Ljava/io/Serializable;)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_HardwareEmulatorInterface_add_1input
( JNIEnv *jni, jobject obj, jstring key, jobject value ) {
    auto name = jni->GetStringUTFChars( key, 0 );
    uint fid = get_autopilot_function_id( name );
    if ( fid == -1 ) {
        std::cout << "Recieved unsupported DataBus input: " << name << std::endl;
        jni->ReleaseStringUTFChars( key, name );
        return;
    }
    jni->ReleaseStringUTFChars( key, name );
    auto &func = AutopilotFunction::autopilot_inputs[fid];
    if ( func.type == VALUE_TYPE::DOUBLE ) {
        static jclass cls = jni->GetObjectClass( value );
        static jmethodID doubleValue = jni->GetMethodID( cls, "doubleValue", "(V)D" );
        
        AutopilotInterface::instance.emulator.get_input( fid ).init( jni->CallDoubleMethod( value, doubleValue ) );
    }
    else if ( func.type == VALUE_TYPE::INT ) {
        static jclass cls = jni->GetObjectClass( value );
        static jmethodID intValue = jni->GetMethodID( cls, "intValue", "(D)I" );
        
        AutopilotInterface::instance.emulator.get_input( fid ).init( jni->CallIntMethod( value, intValue ) );
    }
    else if ( func.type == VALUE_TYPE::DOUBLE_ARRAY ) {
        uint count = jni->GetArrayLength( ( jdoubleArray )value );
        if ( count <= 0 )
            return;
        double *data = jni->GetDoubleArrayElements( ( jdoubleArray )value, 0 );
        
        AutopilotInterface::instance.emulator.get_input( fid ).init( count, data );
        
        jni->ReleaseDoubleArrayElements( ( jdoubleArray )value, data, 0 );
    }
}

/*
* Class:     simulator_integration_HardwareEmulatorInterface
* Method:    exec
* Signature: ()V
*/
JNIEXPORT void JNICALL Java_simulator_integration_HardwareEmulatorInterface_exec
( JNIEnv *, jobject ) {
    AutopilotInterface::instance.emulator.exec();
}


/*
* Class:     simulator_integration_HardwareEmulatorInterface
* Method:    get_outputs
* Signature: (Ljava/util/HashMap;)V
*/
JNIEXPORT void JNICALL Java_simulator_integration_HardwareEmulatorInterface_get_1outputs
( JNIEnv *env, jobject obj, jobject opaque_hashmap ) {
    static jclass cls = env->GetObjectClass( obj );
    static jmethodID add_output = env ->GetMethodID( cls, "add_output",
                                  "(Ljava/util/HashMap;Ljava/lang/String;Ljava/io/Serializable;)V" );
    if ( add_output == 0 )
        return;
        
    for ( auto output_id : Range( AUTOPILOT_OUTPUT_COUNT ) ) {
        jobject key = env->NewStringUTF( AutopilotFunction::autopilot_outputs[output_id].name );
        auto &output = AutopilotInterface::instance.emulator.get_output( output_id );
        if ( output.type == VALUE_TYPE::DOUBLE ) {
            jdouble value = output.double_value;
            env->CallVoidMethod( obj, add_output, opaque_hashmap, key, value );
        }
    }
}

/*
* Class:     simulator_integration_HardwareEmulatorInterface
* Method:    message
* Signature: (Ljava/lang/String;)Ljava/lang/String;
*/
JNIEXPORT jstring JNICALL Java_simulator_integration_HardwareEmulatorInterface_message
( JNIEnv *jni, jobject, jstring string ) {
    auto message = jni->GetStringUTFChars( string, 0 );
    auto res = AutopilotInterface::instance.message( message );
    jni->ReleaseStringUTFChars( string, message );
    return jni->NewStringUTF( res.c_str() );
}