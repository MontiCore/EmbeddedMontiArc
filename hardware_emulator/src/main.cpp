#include "tests.h"



bool test_case( const char *name, bool( *test )( void ) ) {
    Log::white << "Testing " << name  << "\n";
    if ( !test() ) {
        Log::err << "Test failed\n";
        return false;
    }
    Log::test << "Test succeeded\n";
    return true;
}


int main( int argc, char **argv ) {
    ConsoleColor::Console::init();
    //ConsoleColor::Console::test_color();
    
    
    if ( !test_case( "Simple DLL", test_simple_dll ) )
        return 1;
        
    if ( !test_case( "Syscall DLL", test_syscall_dll ) )
        return 2;
        
    if ( !test_case( "Autopilot DLL", test_autopilot_dll ) )
        return 3;
        
    ConsoleColor::Console::drop();
    return 0;
}