/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "tests.h"
#include "utility/utility.h"
#include <iostream>

bool do_debug = false;

TestCase test_cases[] = {
    TestCase( "Simple DLL", test_simple_dll ),
    TestCase( "Function Calling DLL", test_funccalling_dll ),
    //TestCase( "Syscall DLL", test_syscall_dll ), TEMPORARY REMOVED => TODO fix cout symbol dummy
    TestCase( "Hardware Manager querries", test_hardware_manager_querries ),
    TestCase( "ELF read", test_linux_elf_info ),
    TestCase( "Simple ELF", test_simple_elf ),
    TestCase( "Function Calling ELF", test_funccalling_elf ),
    TestCase( "Syscall ELF", test_syscall_elf ),
    TestCase( "ZigZag Autopilot Native", test_zigzag_autopilot_native ),
    TestCase( "ZigZag Autopilot EMU Windows", test_zigzag_autopilot_emu_windows ),
    TestCase( "ZigZag Autopilot EMU Linux", test_zigzag_autopilot_emu_linux ),
    TestCase("EMA Autopilot Native", test_ema_autopilot_native),
    TestCase("EMA Autopilot EMU Windows", test_ema_autopilot_emu_windows),
    TestCase("EMA Autopilot EMU Linux", test_ema_autopilot_emu_linux),
};




int main( int argc, char **argv ) {
    //ConsoleColor::Console::test_color();
    if (argc > 1 && argv[1] == std::string("debug")) {
        std::cout << "Enabling debug output" << std::endl;
        do_debug = true;
    }

    int test_id = 1;
    bool fault = false;
    for ( auto &tc : test_cases ) {
        Log::white.log("Testing %s", tc.name);
        try {
            tc.func();
        }
        catch (std::exception & e) {
            Log::err.log_tag("Test failed:\n\t%s\n\n", e.what());
            fault = true;
            continue;
        }
        ++test_id;
        Log::test.log("Test succeeded\n\n\n");
    }
    if (fault) {
        Log::err.log_tag("Error occured in tests...");
        return -1;
    }
    Log::test.log("All %d tests complete", (test_id - 1));
    return fault ? -1 : 0;
}
