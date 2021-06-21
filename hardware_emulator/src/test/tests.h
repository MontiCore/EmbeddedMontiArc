/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once

extern bool do_debug;

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

void test_linux_elf_info();

void test_simple_elf();

void test_funccalling_elf();

void test_syscall_elf();

void test_zigzag_autopilot_native();
void test_zigzag_autopilot_emu_windows();
void test_zigzag_autopilot_emu_linux();

void test_ema_autopilot_native();
void test_ema_autopilot_emu_windows();
void test_ema_autopilot_emu_linux();