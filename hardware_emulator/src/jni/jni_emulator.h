/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once

#include "computer/memory.h"
#include "computer/computer.h"


struct JNIEmulator {
    Computer *computer;
    MemoryRange jni_interface_slot;
    MemoryRange jni_env_slot;
    MemoryRange buffer_slot;
    
    
    MemorySection *section;
    SectionStack *section_stack;
    
    uint current_array_size;
    ulong current_handle;
    ulong jobject_handle;
    
    JNIEmulator() : computer( nullptr ), current_array_size( 0 ), current_handle( 0 ) {}
    
    bool loaded() {
        return computer != nullptr;
    }
    
    void init( Computer &computer );
    
    bool call( ulong function_addr );
    bool call( ulong function_addr, double param );
    bool call( ulong function_addr, ulong param );
    bool call( ulong function_addr, double *buffer, uint size );
    
    double return_double();
    
    
    static bool get_array_length( Computer &inter, SysCall &syscall );
    static bool get_double_array_elements( Computer &inter, SysCall &syscall );
    static bool release_double_array_elements( Computer &inter, SysCall &syscall );
};
