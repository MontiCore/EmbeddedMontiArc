/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "jni/jni_emulator.h"
#include <jni.h>
#include <stdlib.h>
#include <ctime>


void JNIEmulator::init( Computer &computer ) {
    srand( ( uint ) time( 0 ) );
    this->computer = &computer;
    section = computer.memory.sys_section;
    section_stack = &computer.memory.sys_section_stack;
    
    SystemCalls &sys_calls = computer.sys_calls;
    
    
    
    jni_interface_slot = section_stack->get_range( sizeof( JNINativeInterface_ ) );
    section->annotations.add_annotation( jni_interface_slot, Annotation( "JNINativeInterface", Annotation::SYMBOL ) );
    jni_env_slot = section_stack->get_range( sizeof( JNIEnv ) );
    section->annotations.add_annotation( jni_env_slot, Annotation( "JNIEnv", Annotation::SYMBOL ) );
    buffer_slot = section_stack->get_range( 1024 );
    section->annotations.add_annotation( buffer_slot, Annotation( "JNI buffer", Annotation::SYMBOL ) );
    
    JNINativeInterface_ interf;
    uchar *interface_data = ( uchar * )&interf;
    
#define GET_FUNC_DATA(func_name) *(ulong*)(interface_data + offsetof(JNINativeInterface_, func_name));
    ulong &func_get_array_length = GET_FUNC_DATA( GetArrayLength );
    ulong &func_release_double_array_elements = GET_FUNC_DATA( ReleaseDoubleArrayElements );
    ulong &func_get_double_array_elements = GET_FUNC_DATA( GetDoubleArrayElements );
#undef GET_FUNC_DATA
    
    func_get_array_length =
        sys_calls.add_syscall( SysCall( "GetArrayLength", "JNI", get_array_length, this ) );
    func_release_double_array_elements =
        sys_calls.add_syscall( SysCall( "ReleaseDoubleArrayElements", "JNI", release_double_array_elements, this ) );
    func_get_double_array_elements =
        sys_calls.add_syscall( SysCall( "GetDoubleArrayElements", "JNI", get_double_array_elements, this ) );
        
    computer.memory.write_memory( jni_interface_slot, interface_data );
    
    JNIEnv jnienv = { ( JNINativeInterface_ * )jni_interface_slot.start_address };
    
    computer.memory.write_memory( jni_env_slot, ( uchar * )&jnienv );
    
    jobject_handle = computer.handles.get_handle( "jobject_handle" );
}

bool JNIEmulator::call( ulong function_addr ) {
    computer->fast_call.set_params( jni_env_slot.start_address, jobject_handle );
    return computer->call( function_addr );
}

bool JNIEmulator::call( ulong function_addr, double param ) {
    computer->fast_call.set_params( jni_env_slot.start_address, jobject_handle, *( ulong * ) &param );
    return computer->call( function_addr );
}

bool JNIEmulator::call( ulong function_addr, ulong param ) {
    computer->fast_call.set_params( jni_env_slot.start_address, jobject_handle, param );
    return computer->call( function_addr );
}

bool JNIEmulator::call( ulong function_addr, double *buffer, uint size ) {
    if ( size * sizeof( double ) > buffer_slot.size ) {
        std::cerr << "JNIEmulator::call() with too big array" << std::endl;
        return false;
    }
    computer->memory.write_memory( buffer_slot.start_address, size * sizeof( double ), ( uchar * )buffer );
    current_handle = rand();
    current_array_size = size;
    computer->fast_call.set_params( jni_env_slot.start_address, jobject_handle, current_handle );
    return computer->call( function_addr );
}

double JNIEmulator::return_double() {
    auto ret = computer->fast_call.get_return();
    return *( double * )&ret;
}

bool JNIEmulator::get_array_length( Computer &inter, SysCall &syscall ) {
    auto &jni_emu = *static_cast<JNIEmulator *>( syscall.user_data );
    auto env = inter.fast_call.get_param1();
    auto arr = inter.fast_call.get_param2();
    if ( arr != jni_emu.current_handle )
        std::cerr << "JNIEmulator::get_array_length() on non-local array" << std::endl;
    inter.fast_call.set_return( jni_emu.current_array_size );
    return true;
}

bool JNIEmulator::get_double_array_elements( Computer &inter, SysCall &syscall ) {
    auto &jni_emu = *static_cast<JNIEmulator *>( syscall.user_data );
    auto env = inter.fast_call.get_param1();
    auto arr = inter.fast_call.get_param2();
    if ( arr != jni_emu.current_handle )
        std::cerr << "JNIEmulator::get_double_array_elements() on non-local array" << std::endl;
    inter.fast_call.set_return( jni_emu.buffer_slot.start_address );
    return true;
}

bool JNIEmulator::release_double_array_elements( Computer &inter, SysCall &syscall ) {
    return true;
}
