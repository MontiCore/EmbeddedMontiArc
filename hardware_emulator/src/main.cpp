/* (c) https://github.com/MontiCore/monticore */
#include "tests.h"
#include "utility.h"


TestCase test_cases[] = {
    TestCase( "Filesystem", test_filesystem ),
    TestCase( "Simple DLL", test_simple_dll ),
    TestCase( "Function Calling DLL", test_funccalling_dll ),
    TestCase( "Syscall DLL", test_syscall_dll ),
    TestCase( "Hardware Manager querries", test_hardware_manager_querries ),
    TestCase( "Autopilot DLL", test_autopilot_dll ),
    TestCase( "ELF read", test_linux_elf_info ),
    TestCase( "Simple ELF", test_simple_elf ),
    TestCase( "Function Calling ELF", test_funccalling_elf ),
    TestCase( "Syscall ELF", test_syscall_elf ),
    TestCase( "Autopilot ELF", test_autopilot_elf ),
};




int main( int argc, char **argv ) {
    ConsoleColor::Console::init();
    //ConsoleColor::Console::test_color();
    int test_id = 1;
    for ( auto &tc : test_cases ) {
        Log::white << "Testing " << tc.name << "\n";
        if ( !tc.func() ) {
            Log::err << "Test failed\n";
            return test_id;
        }
        ++test_id;
        Log::test << "Test succeeded\n\n";
    }
    Log::test << "All " << ( test_id - 1 ) << " tests complete\n";
    ConsoleColor::Console::drop();
    return 0;
}
