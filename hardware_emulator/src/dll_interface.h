#pragma once
#include "computer/computer.h"

struct ProgramInterface {
    Computer *computer;
    void init( Computer &computer ) {
        this->computer = &computer;
    }
    Array<uint64_t> addresses;
    bool call_success;
};

namespace ADD_DLL {

    struct Interface : public ProgramInterface {
        enum Functions {
            ADD,
            FUNCTION_COUNT
        };
        bool init( Computer &computer, bool windows );
        int add( int a, int b );
    };
}

namespace LOADED_DLL {

    struct Interface : public ProgramInterface {
        enum Functions {
            TEST_METHOD,
            FUNCTION_COUNT
        };
        bool init( Computer &computer );
        void test_method( void );
    };
}


