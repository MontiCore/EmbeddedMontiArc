/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once

struct TestCase {
    const char *name;
    void( *func )();
    
    TestCase() : name( nullptr ), func( nullptr ) {}
    TestCase( const char *name, void( *func )() ) : name( name ), func( func ) {}
};

void test_simple_dll();

void test_funccalling_dll();

void test_syscall_dll();

void test_hardware_manager_querries();

void test_autopilot_dll();

void test_linux_elf_info();

void test_simple_elf();

void test_funccalling_elf();

void test_syscall_elf();

void test_autopilot_elf();
