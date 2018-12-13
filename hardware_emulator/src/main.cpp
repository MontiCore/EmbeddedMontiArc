#include "tests.h"






int main( int argc, char **argv ) {
    ConsoleColor::Console::init();
    
    if ( !test_simple_dll() )
        return -1;
        
        
    ConsoleColor::Console::drop();
    return 0;
}