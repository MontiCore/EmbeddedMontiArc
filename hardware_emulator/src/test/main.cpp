/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "tests.h"
#include "utility/utility.h"


TestCase test_cases[] = {
    TestCase( "Simple DLL", test_simple_dll ),
    TestCase( "Function Calling DLL", test_funccalling_dll ),
    //TestCase( "Syscall DLL", test_syscall_dll ), TEMPORARY REMOVED => TODO fix cout symbol dummy
    TestCase( "Hardware Manager querries", test_hardware_manager_querries ),
    TestCase( "ELF read", test_linux_elf_info ),
    TestCase( "Simple ELF", test_simple_elf ),
    TestCase( "Function Calling ELF", test_funccalling_elf ),
    TestCase( "Syscall ELF", test_syscall_elf ),
    TestCase( "Autopilot Native", test_autopilot_native ),
    TestCase( "Autopilot EMU Windows", test_autopilot_emu_windows ),
    TestCase( "Autopilot EMU Linux", test_autopilot_emu_linux ),
};




int main( int argc, char **argv ) {
    //ConsoleColor::Console::test_color();
    int test_id = 1;
    bool fault = false;
    for ( auto &tc : test_cases ) {
        Log::white.log("Testing %s", tc.name);
        try {
            tc.func();
        }
        catch (std::exception & e) {
            Log::err.log_tag("Test failed:\n\t%s", e.what());
            fault = true;
            continue;
        }
        ++test_id;
        Log::test.log("Test succeeded\n");
    }
    if (fault) {
        Log::err.log_tag("Error occured in tests...");
        return -1;
    }
    Log::test.log("All %d tests complete", (test_id - 1));
    return fault ? -1 : 0;
}
