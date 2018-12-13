#include "tests.h"






int main( int argc, char **argv ) {
    ConsoleColor::Console::init();
    
    if ( !test_simple_dll() )
        return 1;
        
    if ( !test_syscall_dll() )
        return 2;
        
    ConsoleColor::Console::drop();
    return 0;
}