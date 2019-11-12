/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#include "utility/tests.h"
#include "utility/utility.h"


TestCase test_cases[] = {
    TestCase( "Simple DLL", test_simple_dll ),
    TestCase( "Function Calling DLL", test_funccalling_dll ),
    //TestCase( "Syscall DLL", test_syscall_dll ), TEMPORARY REMOVED => TODO fix cout symbol dummy
    TestCase( "Hardware Manager querries", test_hardware_manager_querries ),
    TestCase( "Autopilot DLL", test_autopilot_dll ),
    TestCase( "ELF read", test_linux_elf_info ),
    TestCase( "Simple ELF", test_simple_elf ),
    TestCase( "Function Calling ELF", test_funccalling_elf ),
    TestCase( "Syscall ELF", test_syscall_elf ),
    TestCase( "Autopilot ELF", test_autopilot_elf ),
};




int main( int argc, char **argv ) {
    //ConsoleColor::Console::test_color();
    int test_id = 1;
    bool fault = false;
    for ( auto &tc : test_cases ) {
        Log::white << "Testing " << tc.name << "\n";
        try {
            tc.func();
        }
        catch (std::exception & e) {
            Log::err << "Test failed:\n\t" << e.what() << "\n";
            fault = true;
            continue;
        }
        ++test_id;
        Log::test << "Test succeeded\n\n";
    }
    if (fault) {
        Log::err << Log::tag << "Error occured in tests...\n";
        return -1;
    }
    Log::test << "All " << ( test_id - 1 ) << " tests complete\n";
    return fault ? -1 : 0;
}
