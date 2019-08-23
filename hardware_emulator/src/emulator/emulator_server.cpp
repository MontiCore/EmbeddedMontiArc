/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
#include "de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_HardwareEmulatorInterface.h"
#include "emulator_manager.h"

/*
    Implementations of the JNI functions that are called from within the RMIModelServer

    add_one_input (Java_simulator_integration_HardwareEmulatorInterface_add_1one_1input)
    must be updated when new data types need to be converted from Java types to C types.
    Same for query_outputs (Java_simulator_integration_HardwareEmulatorInterface_query_1outputs)
    if new data types must be converted from C types to Java types.
*/

JNIEXPORT jboolean JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_init
( JNIEnv *jni, jobject, jstring string1, jstring string2 ) {
    auto config = jni->GetStringUTFChars( string1, 0 );
    auto default_config = jni->GetStringUTFChars( string2, 0 );
    auto res = EmulatorManager::instance.init( config, default_config );
    jni->ReleaseStringUTFChars( string1, config );
    jni->ReleaseStringUTFChars( string2, default_config );
    return res;
}

JNIEXPORT jint JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_alloc_1autopilot
( JNIEnv *jni, jobject, jstring string ) {
    auto config = jni->GetStringUTFChars( string, 0 );
    auto res = EmulatorManager::instance.alloc_emulator( config );
    jni->ReleaseStringUTFChars( string, config );
    return res;
}

JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_free_1autopilot
( JNIEnv *, jobject, jint id ) {
    EmulatorManager::instance.free_emulator( id );
}

JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_execute_1one
( JNIEnv *, jobject, jint id, jlong time_delta ) {
    EmulatorManager::instance.emulators[id]->exec( time_delta );
}

JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_start_1tick
( JNIEnv *, jobject, jlong time_delta ) {
    EmulatorManager::instance.start_tick( time_delta );
}

JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_end_1tick
( JNIEnv *, jobject ) {
    EmulatorManager::instance.end_tick();
}

JNIEXPORT jstring JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_query
( JNIEnv *jni, jobject, jstring string ) {
    auto message = jni->GetStringUTFChars( string, 0 );
    auto res = EmulatorManager::instance.query( message );
    jni->ReleaseStringUTFChars( string, message );
    return jni->NewStringUTF( res.c_str() );
}

JNIEXPORT jstring JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_query_1autopilot
( JNIEnv *jni, jobject, jint id, jstring string ) {
    auto message = jni->GetStringUTFChars( string, 0 );
    auto res = EmulatorManager::instance.emulators[id]->query( message );
    jni->ReleaseStringUTFChars( string, message );
    return jni->NewStringUTF( res.c_str() );
}

JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_add_1one_1input
( JNIEnv *jni, jobject, jint id, jstring key, jobject value ) {
    auto &emulator = *EmulatorManager::instance.emulators[id];
    auto port_name = jni->GetStringUTFChars( key, 0 );
    //Log::info << Log::tag << "Received input for port " << port_name << ": ";
    auto port = emulator.get_port( port_name );
    jni->ReleaseStringUTFChars( key, port_name );
    if ( port == nullptr )
        return;
    auto &port_buffer = port->buffer;
    if ( port_buffer.type == VALUE_TYPE::DOUBLE ) {
        static jclass cls = jni->GetObjectClass( value );
        static jmethodID doubleValue = jni->GetMethodID( cls, "doubleValue", "()D" );
        if ( doubleValue != 0 ) {
            auto d_val = jni->CallDoubleMethod( value, doubleValue );
            //Log::info  << d_val << " (double)\n";
            port_buffer.init( d_val );
        }
        else
            Log::err << "Could not resolve doubleValue method from Java Double\n";
    }
    else if ( port_buffer.type == VALUE_TYPE::INT ) {
        static jclass cls = jni->GetObjectClass( value );
        static jmethodID intValue = jni->GetMethodID( cls, "intValue", "()I" );
        if ( intValue != 0 ) {
            auto i_val = jni->CallIntMethod( value, intValue );
            //Log::info << ": " << i_val << " (int)\n";
            port_buffer.init( (int) i_val );
        }
        else
            Log::err << "Could not resolve intValue method from Java Integer\n";
    }
    else if ( port_buffer.type == VALUE_TYPE::DOUBLE_ARRAY ) {
        uint count = jni->GetArrayLength( ( jdoubleArray )value );
        if ( count <= 0 )
            return;
        double *data = jni->GetDoubleArrayElements( ( jdoubleArray )value, 0 );
        
        /////////////
        /*bool same_val = count == port_buffer.double_array.size;
        if ( same_val ) {
            for ( uint i : urange( count ) ) {
                if ( data[i] != port_buffer.double_array.data[i] ) {
                    same_val = false;
                    break;
                }
            }
        }
        if ( !same_val || true ) {
            Log::info << "{";
            for ( uint i : urange( count ) ) {
                if ( i > 0 )
                    Log::info << ", ";
                Log::info << data[i];
            }
            Log::info << "}\n";
        }*/
        ////////////
        
        port_buffer.init( count, data );
        jni->ReleaseDoubleArrayElements( ( jdoubleArray )value, data, 0 );
    }
}

JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_HardwareEmulatorInterface_query_1outputs
( JNIEnv *env, jobject obj, jint id, jobject opaque_hashmap ) {
    static jclass cls = env->GetObjectClass( obj );
    if ( cls == 0 ) {
        Log::err << Log::tag << "Could not resolve class of HardwareEmulatorInterface\n";
        return;
    }
    static jmethodID add_output = env->GetMethodID( cls, "add_one_output",
                                  "(Ljava/util/HashMap;Ljava/lang/String;Ljava/io/Serializable;)V" );
                                  
    if ( add_output == 0 ) {
        Log::err << Log::tag << "Could not resolve add_one_output java method\n";
        return;
    }
    auto &emulator = *EmulatorManager::instance.emulators[id];
    
    jclass double_class = env->FindClass( "java/lang/Double" );
    if ( double_class == 0 ) {
        Log::err << Log::tag << "Could not resolve Double java class\n";
        return;
    }
    jmethodID double_constructor = env->GetMethodID( double_class, "<init>", "(D)V" );
    if ( double_constructor == 0 ) {
        Log::err << Log::tag << "Could not resolve Double constructor method\n";
        return;
    }
    
    for ( auto &output_port : emulator.output_ports ) {
        if ( output_port.updated ) {
            output_port.updated = false;
            jstring key = env->NewStringUTF( output_port.name.c_str() );
            if ( output_port.buffer.type == VALUE_TYPE::DOUBLE ) {
                jdouble value = output_port.buffer.double_value;
                //Log::info << Log::tag << "Sending port update on port " << output_port.name << ": " << value << "\n";
                auto new_object = env->NewObject( double_class, double_constructor, value );
                env->CallVoidMethod( obj, add_output, opaque_hashmap, key, new_object );
            }
        }
        
    }
}