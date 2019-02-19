#include "emulator_server.h"
#include "emulator_manager.h"


JNIEXPORT jboolean JNICALL Java_simulator_integration_HardwareEmulatorInterface_init
( JNIEnv *, jobject ) {
    return EmulatorManager::instance.init();
}

JNIEXPORT jint JNICALL Java_simulator_integration_HardwareEmulatorInterface_alloc_1autopilot
( JNIEnv *jni, jobject, jstring string ) {
    auto config = jni->GetStringUTFChars( string, 0 );
    auto res = EmulatorManager::instance.alloc_emulator( config );
    jni->ReleaseStringUTFChars( string, config );
    return res;
}

JNIEXPORT void JNICALL Java_simulator_integration_HardwareEmulatorInterface_free_1autopilot
( JNIEnv *, jobject, jint id ) {
    EmulatorManager::instance.free_emulator( id );
}

JNIEXPORT void JNICALL Java_simulator_integration_HardwareEmulatorInterface_start_1tick
( JNIEnv *, jobject, jlong time_delta ) {
    EmulatorManager::instance.start_tick( time_delta );
}

JNIEXPORT void JNICALL Java_simulator_integration_HardwareEmulatorInterface_end_1tick
( JNIEnv *, jobject ) {
    EmulatorManager::instance.end_tick();
}

JNIEXPORT jstring JNICALL Java_simulator_integration_HardwareEmulatorInterface_querry
( JNIEnv *jni, jobject, jstring string ) {
    auto message = jni->GetStringUTFChars( string, 0 );
    auto res = EmulatorManager::instance.querry( message );
    jni->ReleaseStringUTFChars( string, message );
    return jni->NewStringUTF( res.c_str() );
}

JNIEXPORT jstring JNICALL Java_simulator_integration_HardwareEmulatorInterface_querry_1autopilot
( JNIEnv *jni, jobject, jint id, jstring string ) {
    auto message = jni->GetStringUTFChars( string, 0 );
    auto res = EmulatorManager::instance.emulators[id]->querry( message );
    jni->ReleaseStringUTFChars( string, message );
    return jni->NewStringUTF( res.c_str() );
}

JNIEXPORT void JNICALL Java_simulator_integration_HardwareEmulatorInterface_add_1one_1input
( JNIEnv *jni, jobject, jint id, jstring key, jobject value ) {
    auto &emulator = *EmulatorManager::instance.emulators[id];
    auto port_name = jni->GetStringUTFChars( key, 0 );
    int port_id = emulator.get_port_id( port_name );
    jni->ReleaseStringUTFChars( key, port_name );
    if ( port_id < 0 )
        return;
    auto &port_buffer = emulator.input_ports[port_id].buffer;
    if ( port_buffer.type == VALUE_TYPE::DOUBLE ) {
        static jclass cls = jni->GetObjectClass( value );
        static jmethodID doubleValue = jni->GetMethodID( cls, "doubleValue", "(V)D" );
        port_buffer.init( jni->CallDoubleMethod( value, doubleValue ) );
    }
    else if ( port_buffer.type == VALUE_TYPE::INT ) {
        static jclass cls = jni->GetObjectClass( value );
        static jmethodID intValue = jni->GetMethodID( cls, "intValue", "(D)I" );
        port_buffer.init( jni->CallIntMethod( value, intValue ) );
    }
    else if ( port_buffer.type == VALUE_TYPE::DOUBLE_ARRAY ) {
        uint count = jni->GetArrayLength( ( jdoubleArray )value );
        if ( count <= 0 )
            return;
        double *data = jni->GetDoubleArrayElements( ( jdoubleArray )value, 0 );
        port_buffer.init( count, data );
        jni->ReleaseDoubleArrayElements( ( jdoubleArray )value, data, 0 );
    }
}

JNIEXPORT void JNICALL Java_simulator_integration_HardwareEmulatorInterface_querry_1outputs
( JNIEnv *env, jobject obj, jint id, jobject opaque_hashmap ) {
    static jclass cls = env->GetObjectClass( obj );
    static jmethodID add_output = env->GetMethodID( cls, "add_one_output",
                                  "(Ljava/util/HashMap;Ljava/lang/String;Ljava/io/Serializable;)V" );
                                  
    auto &emulator = *EmulatorManager::instance.emulators[id];
    
    for ( auto &output_port : emulator.output_ports ) {
        if ( output_port.updated ) {
            output_port.updated = false;
            jobject key = env->NewStringUTF( output_port.name.c_str() );
            if ( output_port.buffer.type == VALUE_TYPE::DOUBLE ) {
                jdouble value = output_port.buffer.double_value;
                env->CallVoidMethod( obj, add_output, opaque_hashmap, key, value );
            }
        }
        
    }
}