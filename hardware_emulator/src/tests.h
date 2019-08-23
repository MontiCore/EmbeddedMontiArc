/* (c) https://github.com/MontiCore/monticore */
#pragma once

struct TestCase {
    const char *name;
    bool( *func )();
    
    TestCase() : name( nullptr ), func( nullptr ) {}
    TestCase( const char *name, bool( *func )() ) : name( name ), func( func ) {}
};

bool test_filesystem();

bool test_simple_dll();

bool test_funccalling_dll();

bool test_syscall_dll();

bool test_hardware_manager_querries();

bool test_autopilot_dll();

bool test_linux_elf_info();

bool test_simple_elf();

bool test_funccalling_elf();

bool test_syscall_elf();

bool test_autopilot_elf();
