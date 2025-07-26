/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "computer/computer.h"

/*
    The structures implementing TestProgramInterface provide a simple interface to
    an program by resolving and storing the addresses of the functions
    and providing proxy functions that handle the argument passing and calling of the
    emulated function.
*/
struct TestProgramInterface {
    Computer *computer;
    void init( Computer &computer ) {
        this->computer = &computer;
    }
	std::vector<uint64_t> addresses;
};

namespace ADD_DLL {

    struct Interface : public TestProgramInterface {
        enum Functions {
            ADD,
            FUNCTION_COUNT
        };
        void init( Computer &computer, bool windows );
        int add( int a, int b );
    };
}

namespace LOADED_DLL {

    struct Interface : public TestProgramInterface {
        enum Functions {
            TEST_METHOD,
            FUNCTION_COUNT
        };
        void init( Computer &computer );
        void test_method( void );
    };
}


