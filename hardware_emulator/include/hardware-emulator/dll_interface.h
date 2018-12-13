#pragma once
#include "os_windows/os_windows.h"
#include "computer/computer.h"

namespace ADD_DLL {

    struct DllInterface {
        Computer computer;
        OS::Windows os_windows;
        Array<uint64_t> addresses;
        enum Functions {
            ADD,
            FUNCTION_COUNT
        };
        bool test_main();
        void init();
        int add( int a, int b );
    };
    
    
}

namespace LOADED_DLL {

    struct DllInterface {
        Computer computer;
        OS::Windows os_windows;
        Array<uint64_t> addresses;
        enum Functions {
            TEST_METHOD,
            FUNCTION_COUNT
        };
        void test_main();
        void init();
        void test_method( void );
    };
    
    
}


namespace AUTOPILOT_DLL {

    struct DllInterface {
        Computer computer;
        OS::Windows os_windows;
        Array<uint64_t> addresses;
        enum Functions {
            INIT,
            FUNCTION_COUNT
        };
        void test_main();
        void init();
        void init( void *a, void *b );
    };
    
    
}