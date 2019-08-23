/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "computer/computer.h"

/*
    The structures implementing ProgramInterface provide a simple interface to
    an program by resolving and storing the addresses of the functions
    and providing proxy functions that handle the argument passing and calling of the
    emulated function.
*/
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


